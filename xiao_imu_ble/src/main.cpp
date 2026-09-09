/*
 * Xiao nRF52840 Sense Plus — BLE IMU Streamer
 *
 * Reads the onboard LSM6DS3TR-C at ~208 Hz and streams binary
 * packets via BLE notify using the Controller Driver's 29-byte IMU
 * transport contract.
 *
 * Unity path: up to 6 x 29-byte samples (174 bytes) per BLE notification.
 * This reduces notification pressure so 208 samples/sec only needs ~35
 * BLE notifications/sec on high-MTU centrals. Lower-MTU centrals fall back to
 * fewer samples per notification.
 *
 * Packet format per sample (little-endian, 29 bytes):
 *   [uint32_t timestamp_us] [uint8_t sequence] [float gx gy gz ax ay az]
 *
 * Clock sync request (write, 4 bytes):
 *   [uint32_t request_id]
 * Clock sync response (notify, 8 bytes):
 *   [uint32_t request_id] [uint32_t timestamp_us]
 *
 * Gyro units: deg/s   |   Accel units: m/s²
 *
 * Uses the Adafruit nRF52 Bluefruit stack (built into the BSP).
 */

#include <Arduino.h>
#include <bluefruit.h>
#include <LSM6DS3.h>
#include <Wire.h>

static constexpr uint32_t SAMPLE_INTERVAL_US = 4808; // 208 Hz
static constexpr float    G_TO_MS2           = 9.80665f;

// Gyro/accel output registers are contiguous: gyro 0x22–0x27, accel 0x28–0x2D
static constexpr uint8_t  REG_OUTX_L_G      = 0x22;
static constexpr uint8_t  BULK_READ_LEN     = 12;

static constexpr uint8_t  BATCH_CAPACITY     = 6;   // 174 bytes per notify, fits MTU 185+
static constexpr uint16_t BATCH_BYTES        = BATCH_CAPACITY * 29;

// Custom UUIDs — must match the native Controller Driver's XIAO adapter.
static const uint8_t XIAO_SERVICE_UUID128[] = {
    0x34, 0x7a, 0x6f, 0x47, 0x9f, 0xc3, 0x6c, 0x92,
    0x83, 0x45, 0x03, 0x00, 0xe1, 0xb6, 0xe2, 0xbf
};

static const uint8_t XIAO_CHAR_UUID128[] = {
    0x34, 0x7a, 0x6f, 0x47, 0x9f, 0xc3, 0x6c, 0x92,
    0x83, 0x45, 0x04, 0x00, 0xe1, 0xb6, 0xe2, 0xbf
};

static const uint8_t XIAO_SYNC_CHAR_UUID128[] = {
    0x34, 0x7a, 0x6f, 0x47, 0x9f, 0xc3, 0x6c, 0x92,
    0x83, 0x45, 0x05, 0x00, 0xe1, 0xb6, 0xe2, 0xbf
};

BLEService        imuService(XIAO_SERVICE_UUID128);
BLECharacteristic imuChar(XIAO_CHAR_UUID128);
BLECharacteristic syncChar(XIAO_SYNC_CHAR_UUID128);

LSM6DS3 imu(I2C_MODE, 0x6A);

static uint8_t  g_sequence = 0;
static uint32_t g_last_sample_us = 0;
static bool     g_connected = false;
static uint16_t g_conn_handle = BLE_CONN_HANDLE_INVALID;

#pragma pack(push, 1)
struct ImuPacket {
    uint32_t timestamp_us;
    uint8_t  sequence;
    float    gx, gy, gz;
    float    ax, ay, az;
};
#pragma pack(pop)
static_assert(sizeof(ImuPacket) == 29, "Packet must be 29 bytes");

#pragma pack(push, 1)
struct SyncResponse {
    uint32_t request_id;
    uint32_t timestamp_us;
};
#pragma pack(pop)
static_assert(sizeof(SyncResponse) == 8, "Sync response must be 8 bytes");

static ImuPacket g_batch[BATCH_CAPACITY];
static uint8_t   g_batch_count = 0;

// Gyro: raw * 4.375 * (range/125) / 1000 => dps  (range=1000)
static const float gyroScale  = 4.375f * (1000.0f / 125.0f) / 1000.0f;
// Accel: raw * 0.061 * (range>>1) / 1000 => g     (range=8)
static const float accelScale = 0.061f * (8 >> 1) / 1000.0f;

// --- BLE callbacks ---

void connect_cb(uint16_t conn_handle) {
    g_connected = true;
    g_conn_handle = conn_handle;
    g_batch_count = 0;
    Serial.println("BLE connected");

    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn) {
        conn->requestConnectionParameter(12); // 15 ms (more Android-friendly)
        conn->requestMtuExchange(247);
    }
}

void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    g_connected = false;
    g_conn_handle = BLE_CONN_HANDLE_INVALID;
    g_batch_count = 0;
    Serial.println("BLE disconnected");
}

void sync_write_cb(
    uint16_t conn_handle,
    BLECharacteristic* characteristic,
    uint8_t* data,
    uint16_t len
) {
    // Capture the device clock before parsing or doing any other work.
    const uint32_t received_at_us = micros();
    (void)conn_handle;
    (void)characteristic;
    if (len != sizeof(uint32_t)) {
        return;
    }

    SyncResponse response;
    memcpy(&response.request_id, data, sizeof(response.request_id));
    response.timestamp_us = received_at_us;
    syncChar.notify(
        reinterpret_cast<uint8_t*>(&response),
        sizeof(response)
    );
}

static uint8_t currentBatchTarget() {
    if (!g_connected || g_conn_handle == BLE_CONN_HANDLE_INVALID) {
        return 0;
    }

    BLEConnection* conn = Bluefruit.Connection(g_conn_handle);
    if (!conn) {
        return 0;
    }

    uint16_t mtu = conn->getMtu();
    if (mtu <= 3) {
        return 0;
    }

    uint16_t payload_bytes = mtu - 3;
    if (payload_bytes < sizeof(ImuPacket)) {
        return 0;
    }

    uint8_t samples = static_cast<uint8_t>(payload_bytes / sizeof(ImuPacket));
    if (samples > BATCH_CAPACITY) {
        samples = BATCH_CAPACITY;
    }
    return samples;
}

// --- Setup ---

void setup() {
    Serial.begin(115200);

    pinMode(LED_RED, OUTPUT);
    uint32_t start = millis();
    while (!Serial && millis() - start < 2000) {
        digitalToggle(LED_RED);
        delay(100);
    }
    digitalWrite(LED_RED, HIGH);

    Serial.println("XiaoIMU BLE Streamer (208 Hz, +/-8g, +/-1000dps, 174-byte notify)");

    // --- IMU ---
    imu.settings.gyroEnabled  = 1;
    imu.settings.gyroRange    = 1000;
    imu.settings.gyroSampleRate = 208;
    imu.settings.accelEnabled = 1;
    imu.settings.accelRange   = 8;
    imu.settings.accelSampleRate = 208;

    if (imu.begin() != 0) {
        Serial.println("ERROR: IMU init failed");
        while (1) {
            digitalToggle(LED_RED);
            delay(200);
        }
    }
    Serial.println("IMU ready (208 Hz, +/-8g, +/-1000dps)");

    // --- BLE ---
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("XiaoIMU");
    Bluefruit.Periph.setConnectCallback(connect_cb);
    Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
    Bluefruit.Periph.setConnInterval(6, 24); // 7.5–30 ms (wider range for Android tolerance)

    // Service
    imuService.begin();

    // Characteristic — batched 29-byte packets per notification, notify only
    imuChar.setProperties(CHR_PROPS_NOTIFY);
    imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    imuChar.setMaxLen(BATCH_BYTES);
    imuChar.begin();

    // Clock synchronization — write request ID, notify request ID + micros().
    syncChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
    syncChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    syncChar.setMaxLen(sizeof(SyncResponse));
    syncChar.setWriteCallback(sync_write_cb);
    syncChar.begin();

    // Advertising
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(imuService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    Serial.println("Advertising as 'XiaoIMU'...");
    g_last_sample_us = micros();
}

// --- Main loop ---

void loop() {
    uint32_t now = micros();
    if (now - g_last_sample_us < SAMPLE_INTERVAL_US) {
        return;
    }
    g_last_sample_us += SAMPLE_INTERVAL_US;

    // Read even when not connected to keep the sensor's output register fresh
    uint8_t buf[BULK_READ_LEN];
    if (imu.readRegisterRegion(buf, REG_OUTX_L_G, BULK_READ_LEN) != 0) {
        return;
    }

    if (!g_connected) {
        return;
    }

    uint8_t batch_target = currentBatchTarget();
    if (batch_target == 0) {
        return;
    }

    auto to_i16 = [](uint8_t lo, uint8_t hi) -> int16_t {
        return static_cast<int16_t>(lo | (hi << 8));
    };

    ImuPacket& pkt = g_batch[g_batch_count];
    pkt.timestamp_us = now;
    pkt.sequence     = g_sequence++;
    pkt.gx = to_i16(buf[0],  buf[1])  * gyroScale;
    pkt.gy = to_i16(buf[2],  buf[3])  * gyroScale;
    pkt.gz = to_i16(buf[4],  buf[5])  * gyroScale;
    pkt.ax = to_i16(buf[6],  buf[7])  * accelScale * G_TO_MS2;
    pkt.ay = to_i16(buf[8],  buf[9])  * accelScale * G_TO_MS2;
    pkt.az = to_i16(buf[10], buf[11]) * accelScale * G_TO_MS2;
    g_batch_count++;

    if (g_batch_count >= batch_target) {
        imuChar.notify(
            reinterpret_cast<uint8_t*>(g_batch),
            g_batch_count * sizeof(ImuPacket)
        );
        g_batch_count = 0;
    }
}
