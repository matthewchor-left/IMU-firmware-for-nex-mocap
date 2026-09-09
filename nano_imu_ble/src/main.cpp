/*
 * Arduino Nano 33 BLE Sense Rev2 — BLE 9DoF IMU Streamer
 *
 * Reads the onboard BMI270 and BMM150 using Arduino_BMI270_BMM150.
 * Accel/gyro retain the Controller Driver's 29-byte transport packet, while
 * compensated magnetometer readings use a separate 17-byte BLE characteristic.
 *
 * Packet format (little-endian, 29 bytes):
 *   [uint32_t timestamp_us] [uint8_t sequence] [float gx gy gz ax ay az]
 *
 * Gyro units: deg/s   |   Accel units: m/s²
 *
 * Magnetometer packet (little-endian, 17 bytes):
 *   [uint32_t timestamp_us] [uint8_t sequence] [float mx my mz]
 *
 * Magnetometer units: microtesla (uT)
 *
 * IMPLEMENTATION NOTES:
 *
 * 1. Arduino_BMI270_BMM150 1.2.3 works with ArduinoBLE 2.1.0 when IMU.begin()
 *    completes before BLE.begin(). A 15-second macOS notification test remains
 *    connected while receiving both characteristics.
 *
 * 2. The BMI270 requires an 8KB microcode config blob uploaded via I2C
 *    before it will produce any sensor data (outputs zeros without it).
 *    The blob is extracted from the Bosch BMI270-Sensor-API and embedded
 *    in bmi270_config.h.
 *
 * 3. The retained raw-I2C fallback uploads that blob in chunks with
 *    INIT_ADDR_0/1 updated between bursts.
 *    set between each burst (datasheet Section 5.2.65-66). Without updating
 *    the address registers, the internal init validation fails.
 *
 * 4. The Nano 33 BLE requires PIN_ENABLE_SENSORS_3V3 (pin 33) and
 *    PIN_ENABLE_I2C_PULLUP (pin 32) set HIGH before any I2C communication.
 *    Without this, Wire1.begin() hangs indefinitely because the BMI270
 *    is unpowered and holds SDA low.
 *
 * 5. BMI270 data register layout (datasheet Section 4.6):
 *    DATA_8..DATA_13  (0x0C-0x11) = Accelerometer X, Y, Z
 *    DATA_14..DATA_19 (0x12-0x17) = Gyroscope X, Y, Z
 *    (NOT the other way around — easy to confuse)
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#undef BMM150_REG_CHIP_ID
#undef BMM150_REG_DATA_X_LSB
#undef BMM150_REG_POWER_CONTROL
#undef BMM150_REG_OP_MODE
#undef BMM150_REG_REP_XY
#undef BMM150_REG_REP_Z
#include "bmi270_config.h"

// --- BMI270 Register Map ---
static constexpr uint8_t BMI270_ADDR         = 0x68;
static constexpr uint8_t REG_CHIP_ID         = 0x00;  // Expected: 0x24
static constexpr uint8_t REG_ERR             = 0x02;
static constexpr uint8_t REG_STATUS          = 0x03;
static constexpr uint8_t REG_CMD             = 0x7E;
static constexpr uint8_t REG_PWR_CONF        = 0x7C;
static constexpr uint8_t REG_PWR_CTRL        = 0x7D;
static constexpr uint8_t REG_INIT_CTRL       = 0x59;
static constexpr uint8_t REG_INIT_ADDR_0     = 0x5B;  // Config upload word-address low nibble
static constexpr uint8_t REG_INIT_ADDR_1     = 0x5C;  // Config upload word-address high byte
static constexpr uint8_t REG_INIT_DATA       = 0x5E;
static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
static constexpr uint8_t REG_ACC_CONF        = 0x40;
static constexpr uint8_t REG_ACC_RANGE       = 0x41;
static constexpr uint8_t REG_GYR_CONF        = 0x42;
static constexpr uint8_t REG_GYR_RANGE       = 0x43;
static constexpr uint8_t REG_DATA_8          = 0x0C;  // Accel X LSB (gyro starts at 0x12)

// --- BMM150 Register Map ---
static constexpr uint8_t BMM150_ADDR              = 0x10;
static constexpr uint8_t BMM150_REG_CHIP_ID       = 0x40;  // Expected: 0x32
static constexpr uint8_t BMM150_REG_DATA_X_LSB    = 0x42;
static constexpr uint8_t BMM150_REG_POWER_CONTROL = 0x4B;
static constexpr uint8_t BMM150_REG_OP_MODE       = 0x4C;
static constexpr uint8_t BMM150_REG_REP_XY        = 0x51;
static constexpr uint8_t BMM150_REG_REP_Z         = 0x52;
static constexpr uint8_t BMM150_REG_DIG_X1        = 0x5D;
static constexpr uint8_t BMM150_REG_DIG_Z4_LSB    = 0x62;
static constexpr uint8_t BMM150_REG_DIG_Z2_LSB    = 0x68;

// Sensitivities: ±2000 dps gyro, ±4g accel
static constexpr float GYRO_SENSITIVITY  = 2000.0f / 32768.0f;  // deg/s per LSB
static constexpr float ACCEL_SENSITIVITY = 4.0f / 32768.0f;     // g per LSB
static constexpr float G_TO_MS2          = 9.80665f;

static constexpr uint32_t SAMPLE_INTERVAL_US = 10000;  // 100 Hz
static constexpr uint32_t MAG_INTERVAL_US    = 50000;  // 20 Hz
static constexpr uint32_t SERIAL_INTERVAL_US = 20000;  // 50 Hz

// BLE UUIDs — unique to Nano, distinct from Xiao (0003/0004)
static const char* SERVICE_UUID = "bfe2b6e1-0005-4583-926c-c39f476f7a34";
static const char* CHAR_UUID    = "bfe2b6e1-0006-4583-926c-c39f476f7a34";
static const char* MAG_CHAR_UUID = "bfe2b6e1-0007-4583-926c-c39f476f7a34";

BLEService        imuService(SERVICE_UUID);
BLECharacteristic imuChar(CHAR_UUID, BLERead | BLENotify, 29);
BLECharacteristic magChar(MAG_CHAR_UUID, BLERead | BLENotify, 17);

static uint8_t  g_sequence       = 0;
static uint8_t  g_mag_sequence   = 0;
static uint32_t g_last_sample_us = 0;
static uint32_t g_last_mag_us    = 0;
static uint32_t g_last_diagnostic_us = 0;
static bool     g_imu_ok         = false;
static bool     g_mag_ok         = false;
static bool     g_wire_started    = false;
static float    g_latest_mx      = 0.0f;
static float    g_latest_my      = 0.0f;
static float    g_latest_mz      = 0.0f;
static float    g_latest_gx      = 0.0f;
static float    g_latest_gy      = 0.0f;
static float    g_latest_gz      = 0.0f;
static float    g_latest_ax      = 0.0f;
static float    g_latest_ay      = 0.0f;
static float    g_latest_az      = 0.0f;
static uint8_t  g_zero_sample_count = 0;

#pragma pack(push, 1)
struct ImuPacket {
    uint32_t timestamp_us;
    uint8_t  sequence;
    float    gx, gy, gz;
    float    ax, ay, az;
};

struct MagPacket {
    uint32_t timestamp_us;
    uint8_t  sequence;
    float    mx, my, mz;
};
#pragma pack(pop)
static_assert(sizeof(ImuPacket) == 29, "Packet must be 29 bytes");
static_assert(sizeof(MagPacket) == 17, "Mag packet must be 17 bytes");

struct Bmm150Trim {
    int8_t   x1;
    int8_t   y1;
    int8_t   x2;
    int8_t   y2;
    uint16_t z1;
    int16_t  z2;
    int16_t  z3;
    int16_t  z4;
    uint8_t  xy1;
    int8_t   xy2;
    uint16_t xyz1;
};

static Bmm150Trim g_bmm150_trim{};

// --- Raw I2C helpers (Wire1 = internal sensor bus on Nano 33 BLE) ---

static void bmi270_write(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
}

static uint8_t bmi270_read(uint8_t reg) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission();
    Wire1.requestFrom(BMI270_ADDR, (uint8_t)1);
    return Wire1.read();
}

static void bmi270_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire1.beginTransmission(BMI270_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission();
    Wire1.requestFrom(BMI270_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire1.read();
    }
}

static bool i2c_write(uint8_t address, uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(address);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

static bool i2c_read_burst(uint8_t address, uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire1.beginTransmission(address);
    Wire1.write(reg);
    if (Wire1.endTransmission() != 0) return false;
    if (Wire1.requestFrom(address, len) != len) return false;
    for (uint8_t i = 0; i < len; ++i) {
        buf[i] = Wire1.read();
    }
    return true;
}

/*
 * Upload the 8KB BMI270 config blob in 128-byte bursts.
 *
 * Per BMI270 datasheet (Section 5.2.65-66): when the host cannot write
 * the full 8KB in a single burst, INIT_ADDR_0 and INIT_ADDR_1 must be
 * set to (byte_offset / 2) before each chunk. This is a word address
 * split across two registers: low nibble in ADDR_0, high byte in ADDR_1.
 *
 * The Wire buffer on nRF52840 Mbed is 256 bytes, but we use 128-byte
 * bursts for reliability (1 reg byte + 128 data = 129 bytes per txn).
 */
static bool bmi270_upload_config() {
    static constexpr uint16_t BURST_LEN = 128;

    bmi270_write(REG_PWR_CONF, 0x00);  // Disable advanced power save for config
    delay(1);
    bmi270_write(REG_INIT_CTRL, 0x00);  // Prepare for config upload

    const uint16_t config_size = sizeof(bmi270_config_file);
    for (uint16_t offset = 0; offset < config_size; offset += BURST_LEN) {
        uint16_t word_addr = offset / 2;
        bmi270_write(REG_INIT_ADDR_0, (uint8_t)(word_addr & 0x0F));
        bmi270_write(REG_INIT_ADDR_1, (uint8_t)((word_addr >> 4) & 0xFF));

        uint16_t chunk = min(BURST_LEN, (uint16_t)(config_size - offset));
        Wire1.beginTransmission(BMI270_ADDR);
        Wire1.write(REG_INIT_DATA);
        Wire1.write(&bmi270_config_file[offset], chunk);
        Wire1.endTransmission();
    }

    bmi270_write(REG_INIT_CTRL, 0x01);  // Signal config upload complete
    delay(150);  // Wait for internal initialization to finish

    uint8_t status = bmi270_read(REG_INTERNAL_STATUS);
    return (status & 0x01) != 0;  // Bit 0 = init OK
}

static bool bmi270_init() {
    // CRITICAL: enable 3V3 power to onboard sensors and I2C pull-ups.
    // Without this, the BMI270 is unpowered and Wire1.begin() hangs
    // because SDA is held low by the unpowered chip.
    pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
    pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
    digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
    digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
    delay(200);  // Generous settling time for sensor power rail

    if (!g_wire_started) {
        Wire1.begin();
        Wire1.setClock(400000);  // 400 kHz fast-mode I2C
        g_wire_started = true;
        delay(50);
    }

    // Soft-reset BMI270
    bmi270_write(REG_CMD, 0xB6);
    delay(100);  // Datasheet says 2ms min; generous after cold power-up

    // Chip ID read with retry — first read after reset can return 0x00
    uint8_t chip_id = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        chip_id = bmi270_read(REG_CHIP_ID);
        if (chip_id == 0x24) break;
        delay(20);
    }
    Serial.print("BMI270 chip ID: 0x");
    Serial.println(chip_id, HEX);
    if (chip_id != 0x24) {
        Serial.println("ERROR: unexpected chip ID");
        return false;
    }

    if (!bmi270_upload_config()) {
        Serial.println("ERROR: config upload failed");
        return false;
    }
    Serial.println("BMI270 config loaded");

    // Accel: 100 Hz ODR, normal filter, perf mode, ±4g
    bmi270_write(REG_ACC_CONF, 0xA8);   // odr=100Hz, bwp=normal, filter_perf=1
    bmi270_write(REG_ACC_RANGE, 0x01);  // ±4g

    // Gyro: 100 Hz ODR, normal filter, ±2000 dps
    bmi270_write(REG_GYR_CONF, 0xA9);   // odr=100Hz, bwp=normal, noise_perf=1
    bmi270_write(REG_GYR_RANGE, 0x00);  // ±2000 dps

    // Enable accel + gyro + temp acquisition; keep auxiliary interface off.
    bmi270_write(REG_PWR_CTRL, 0x0E);   // temp=1, acc=1, gyr=1, aux=0
    bmi270_write(REG_PWR_CONF, 0x00);   // Disable advanced power save
    delay(50);  // Wait for sensors to start producing data

    Serial.println("BMI270 ready (100Hz, +/-4g, +/-2000dps)");
    return true;
}

/*
 * Read 12 bytes starting at DATA_8 (0x0C):
 *   bytes 0-5:  accel X, Y, Z (16-bit signed LE) — registers 0x0C..0x11
 *   bytes 6-11: gyro  X, Y, Z (16-bit signed LE) — registers 0x12..0x17
 */
static void bmi270_read_imu(float* gx, float* gy, float* gz,
                            float* ax, float* ay, float* az) {
    uint8_t buf[12];
    bmi270_read_burst(REG_DATA_8, buf, 12);

    int16_t raw_ax = (int16_t)(buf[0]  | (buf[1]  << 8));
    int16_t raw_ay = (int16_t)(buf[2]  | (buf[3]  << 8));
    int16_t raw_az = (int16_t)(buf[4]  | (buf[5]  << 8));
    int16_t raw_gx = (int16_t)(buf[6]  | (buf[7]  << 8));
    int16_t raw_gy = (int16_t)(buf[8]  | (buf[9]  << 8));
    int16_t raw_gz = (int16_t)(buf[10] | (buf[11] << 8));

    *gx = raw_gx * GYRO_SENSITIVITY;
    *gy = raw_gy * GYRO_SENSITIVITY;
    *gz = raw_gz * GYRO_SENSITIVITY;
    *ax = raw_ax * ACCEL_SENSITIVITY * G_TO_MS2;
    *ay = raw_ay * ACCEL_SENSITIVITY * G_TO_MS2;
    *az = raw_az * ACCEL_SENSITIVITY * G_TO_MS2;
}

static uint16_t uint16_le(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

static int16_t int16_le(const uint8_t* bytes) {
    return static_cast<int16_t>(uint16_le(bytes));
}

static bool bmm150_init() {
    // Bring the BMM150 out of suspend mode. It is a separate Wire1 device,
    // so the BMI270 auxiliary interface remains disabled.
    if (!i2c_write(BMM150_ADDR, BMM150_REG_POWER_CONTROL, 0x01)) return false;
    delay(3);

    uint8_t chip_id = 0;
    if (!i2c_read_burst(BMM150_ADDR, BMM150_REG_CHIP_ID, &chip_id, 1)) return false;
    Serial.print("BMM150 chip ID: 0x");
    Serial.println(chip_id, HEX);
    if (chip_id != 0x32) return false;

    uint8_t x1_y1[2]{};
    uint8_t z4_x2_y2[4]{};
    uint8_t z2_to_xy1[10]{};
    if (!i2c_read_burst(BMM150_ADDR, BMM150_REG_DIG_X1, x1_y1, sizeof(x1_y1)) ||
        !i2c_read_burst(BMM150_ADDR, BMM150_REG_DIG_Z4_LSB, z4_x2_y2, sizeof(z4_x2_y2)) ||
        !i2c_read_burst(BMM150_ADDR, BMM150_REG_DIG_Z2_LSB, z2_to_xy1, sizeof(z2_to_xy1))) {
        return false;
    }

    g_bmm150_trim.x1 = static_cast<int8_t>(x1_y1[0]);
    g_bmm150_trim.y1 = static_cast<int8_t>(x1_y1[1]);
    g_bmm150_trim.x2 = static_cast<int8_t>(z4_x2_y2[2]);
    g_bmm150_trim.y2 = static_cast<int8_t>(z4_x2_y2[3]);
    g_bmm150_trim.z1 = uint16_le(&z2_to_xy1[2]);
    g_bmm150_trim.z2 = int16_le(&z2_to_xy1[0]);
    g_bmm150_trim.z3 = int16_le(&z2_to_xy1[6]);
    g_bmm150_trim.z4 = int16_le(&z4_x2_y2[0]);
    g_bmm150_trim.xy1 = z2_to_xy1[9];
    g_bmm150_trim.xy2 = static_cast<int8_t>(z2_to_xy1[8]);
    g_bmm150_trim.xyz1 =
        static_cast<uint16_t>((static_cast<uint16_t>(z2_to_xy1[5] & 0x7F) << 8) |
                              z2_to_xy1[4]);

    // Regular repetitions and 20 Hz ODR, normal power mode.
    if (!i2c_write(BMM150_ADDR, BMM150_REG_REP_XY, 0x04) ||
        !i2c_write(BMM150_ADDR, BMM150_REG_REP_Z, 0x07) ||
        !i2c_write(BMM150_ADDR, BMM150_REG_OP_MODE, 0x28)) {
        return false;
    }
    delay(50);
    Serial.println("BMM150 ready (20Hz, compensated uT)");
    return true;
}

static float bmm150_compensate_xy(int16_t raw, uint16_t rhall,
                                  int8_t axis1, int8_t axis2) {
    if (raw == -4096 || rhall == 0 || g_bmm150_trim.xyz1 == 0) return 0.0f;
    const float ratio =
        static_cast<float>(g_bmm150_trim.xyz1) * 16384.0f / static_cast<float>(rhall) -
        16384.0f;
    const float correction =
        static_cast<float>(g_bmm150_trim.xy2) * ratio * ratio / 268435456.0f +
        ratio * static_cast<float>(g_bmm150_trim.xy1) / 16384.0f;
    const float sensitivity = static_cast<float>(axis2) + 160.0f;
    return ((static_cast<float>(raw) * (correction + 256.0f) * sensitivity /
             8192.0f) +
            static_cast<float>(axis1) * 8.0f) /
           16.0f;
}

static float bmm150_compensate_z(int16_t raw, uint16_t rhall) {
    if (raw == -16384 || rhall == 0 || g_bmm150_trim.z1 == 0 ||
        g_bmm150_trim.z2 == 0 || g_bmm150_trim.xyz1 == 0) {
        return 0.0f;
    }
    const float numerator =
        (static_cast<float>(raw) - static_cast<float>(g_bmm150_trim.z4)) *
            131072.0f -
        static_cast<float>(g_bmm150_trim.z3) *
            (static_cast<float>(rhall) - static_cast<float>(g_bmm150_trim.xyz1));
    const float denominator =
        (static_cast<float>(g_bmm150_trim.z2) +
         static_cast<float>(g_bmm150_trim.z1) * static_cast<float>(rhall) /
             32768.0f) *
        4.0f;
    return (numerator / denominator) / 16.0f;
}

static bool bmm150_read_magnetic_field(float* mx, float* my, float* mz) {
    uint8_t data[8]{};
    if (!i2c_read_burst(BMM150_ADDR, BMM150_REG_DATA_X_LSB, data, sizeof(data))) {
        return false;
    }

    const int16_t raw_x =
        static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[1])) * 32 |
                             (data[0] >> 3));
    const int16_t raw_y =
        static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[3])) * 32 |
                             (data[2] >> 3));
    const int16_t raw_z =
        static_cast<int16_t>(static_cast<int16_t>(static_cast<int8_t>(data[5])) * 128 |
                             (data[4] >> 1));
    const uint16_t rhall =
        static_cast<uint16_t>((static_cast<uint16_t>(data[7]) << 6) | (data[6] >> 2));

    *mx = bmm150_compensate_xy(raw_x, rhall, g_bmm150_trim.x1, g_bmm150_trim.x2);
    *my = bmm150_compensate_xy(raw_y, rhall, g_bmm150_trim.y1, g_bmm150_trim.y2);
    *mz = bmm150_compensate_z(raw_z, rhall);
    return true;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    delay(1000);  // Board stabilization; no blocking on Serial

    Serial.println("NanoIMU 9DoF BLE Streamer");

    if (!IMU.begin()) {
        Serial.println("FATAL: official 9DoF IMU initialization failed");
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
    }
    g_imu_ok = true;
    g_mag_ok = true;

#ifdef NANO_USB_ORIENTATION_VIEWER
    Serial.println("USB orientation viewer mode (official 9DoF driver, BLE disabled)");
    g_last_sample_us = micros();
    g_last_mag_us = g_last_sample_us;
    g_last_diagnostic_us = g_last_sample_us;
    return;
#endif

#ifndef NANO_USB_ORIENTATION_VIEWER
    // Initialize BLE after the combined IMU driver.
    if (!BLE.begin()) {
        Serial.println("FATAL: BLE init failed");
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); }
    }

    BLE.setLocalName("NanoIMU");
    BLE.setDeviceName("NanoIMU");
    BLE.setAdvertisedService(imuService);
    BLE.setConnectable(true);
    imuService.addCharacteristic(imuChar);
    imuService.addCharacteristic(magChar);
    BLE.addService(imuService);
    BLE.advertise();

    Serial.println("Advertising as 'NanoIMU'");
#else
    Serial.println("USB orientation viewer mode (BLE disabled)");
#endif
    g_last_sample_us = micros();
    g_last_mag_us = g_last_sample_us;
    g_last_diagnostic_us = g_last_sample_us;
}

void loop() {
#ifdef NANO_USB_ORIENTATION_VIEWER
    const uint32_t now = micros();
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(g_latest_mx, g_latest_my, g_latest_mz);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(g_latest_gx, g_latest_gy, g_latest_gz);
    }
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(g_latest_ax, g_latest_ay, g_latest_az);
    }
    if (now - g_last_diagnostic_us >= SERIAL_INTERVAL_US) {
        g_last_diagnostic_us += SERIAL_INTERVAL_US;

        Serial.print("9DOF,");
        Serial.print(now);
        Serial.print(',');
        Serial.print(g_latest_gx, 4);
        Serial.print(',');
        Serial.print(g_latest_gy, 4);
        Serial.print(',');
        Serial.print(g_latest_gz, 4);
        Serial.print(',');
        Serial.print(g_latest_ax * G_TO_MS2, 4);
        Serial.print(',');
        Serial.print(g_latest_ay * G_TO_MS2, 4);
        Serial.print(',');
        Serial.print(g_latest_az * G_TO_MS2, 4);
        Serial.print(',');
        Serial.print(g_latest_mx, 4);
        Serial.print(',');
        Serial.print(g_latest_my, 4);
        Serial.print(',');
        Serial.println(g_latest_mz, 4);
    }
#else
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected: ");
        Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH);

        g_last_sample_us = micros();
        g_last_mag_us = g_last_sample_us;
        g_sequence = 0;
        g_mag_sequence = 0;

        while (central.connected()) {
            uint32_t now = micros();
            if (IMU.gyroscopeAvailable()) {
                IMU.readGyroscope(g_latest_gx, g_latest_gy, g_latest_gz);
            }
            if (IMU.accelerationAvailable()) {
                IMU.readAcceleration(g_latest_ax, g_latest_ay, g_latest_az);
            }
            if (IMU.magneticFieldAvailable()) {
                IMU.readMagneticField(g_latest_mx, g_latest_my, g_latest_mz);
            }
            if (now - g_last_sample_us >= SAMPLE_INTERVAL_US) {
                g_last_sample_us += SAMPLE_INTERVAL_US;

                ImuPacket pkt;
                pkt.timestamp_us = now;
                pkt.sequence = g_sequence++;

                pkt.gx = g_latest_gx;
                pkt.gy = g_latest_gy;
                pkt.gz = g_latest_gz;
                pkt.ax = g_latest_ax * G_TO_MS2;
                pkt.ay = g_latest_ay * G_TO_MS2;
                pkt.az = g_latest_az * G_TO_MS2;

                imuChar.writeValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
            }

            if (g_mag_ok && now - g_last_mag_us >= MAG_INTERVAL_US) {
                g_last_mag_us += MAG_INTERVAL_US;
                MagPacket pkt;
                pkt.timestamp_us = now;
                pkt.sequence = g_mag_sequence++;
                pkt.mx = g_latest_mx;
                pkt.my = g_latest_my;
                pkt.mz = g_latest_mz;
                magChar.writeValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
            }
        }

        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Disconnected");
    } else {
        const uint32_t now = micros();
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(g_latest_gx, g_latest_gy, g_latest_gz);
        }
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(g_latest_ax, g_latest_ay, g_latest_az);
        }
        if (IMU.magneticFieldAvailable()) {
            IMU.readMagneticField(g_latest_mx, g_latest_my, g_latest_mz);
        }
        if (g_imu_ok && g_mag_ok && now - g_last_diagnostic_us >= SERIAL_INTERVAL_US) {
            g_last_diagnostic_us += SERIAL_INTERVAL_US;

            Serial.print("9DOF,");
            Serial.print(now);
            Serial.print(',');
            Serial.print(g_latest_gx, 4);
            Serial.print(',');
            Serial.print(g_latest_gy, 4);
            Serial.print(',');
            Serial.print(g_latest_gz, 4);
            Serial.print(',');
            Serial.print(g_latest_ax * G_TO_MS2, 4);
            Serial.print(',');
            Serial.print(g_latest_ay * G_TO_MS2, 4);
            Serial.print(',');
            Serial.print(g_latest_az * G_TO_MS2, 4);
            Serial.print(',');
            Serial.print(g_latest_mx, 4);
            Serial.print(',');
            Serial.print(g_latest_my, 4);
            Serial.print(',');
            Serial.println(g_latest_mz, 4);
        }
        if (now - g_last_diagnostic_us > SERIAL_INTERVAL_US * 4) {
            // Recover after a long BLE connection without flooding stale samples.
            g_last_diagnostic_us = now;
        }
    }
#endif
}
