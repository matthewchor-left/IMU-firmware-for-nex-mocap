#include "transport_ble.h"

#include <Arduino.h>
#include <bluefruit.h>

static constexpr uint8_t BATCH_CAPACITY = 6;
static constexpr uint16_t BATCH_BYTES = BATCH_CAPACITY * IMU_PACKET_SIZE;

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

static BLEService imuService(XIAO_SERVICE_UUID128);
static BLECharacteristic imuChar(XIAO_CHAR_UUID128);
static BLECharacteristic syncChar(XIAO_SYNC_CHAR_UUID128);

static bool g_connected = false;
static uint16_t g_conn_handle = BLE_CONN_HANDLE_INVALID;
static ImuPacket g_batch[BATCH_CAPACITY];
static uint8_t g_batch_count = 0;

static void connect_cb(uint16_t conn_handle) {
    g_connected = true;
    g_conn_handle = conn_handle;
    g_batch_count = 0;

    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn) {
        conn->requestConnectionParameter(12);
        conn->requestMtuExchange(247);
    }
}

static void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    g_connected = false;
    g_conn_handle = BLE_CONN_HANDLE_INVALID;
    g_batch_count = 0;
}

static void sync_write_cb(
    uint16_t conn_handle,
    BLECharacteristic* characteristic,
    uint8_t* data,
    uint16_t len
) {
    const uint32_t received_at_us = micros();
    (void)conn_handle;
    (void)characteristic;
    if (len != sizeof(uint32_t)) {
        return;
    }

    SyncResponse response;
    memcpy(&response.request_id, data, sizeof(response.request_id));
    response.timestamp_us = received_at_us;
    syncChar.notify(reinterpret_cast<uint8_t*>(&response), sizeof(response));
}

static uint8_t current_batch_target() {
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
    if (payload_bytes < IMU_PACKET_SIZE) {
        return 0;
    }

    uint8_t samples = static_cast<uint8_t>(payload_bytes / IMU_PACKET_SIZE);
    if (samples > BATCH_CAPACITY) {
        samples = BATCH_CAPACITY;
    }
    return samples;
}

void transport_ble_init() {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("XiaoIMU");
    Bluefruit.Periph.setConnectCallback(connect_cb);
    Bluefruit.Periph.setDisconnectCallback(disconnect_cb);
    Bluefruit.Periph.setConnInterval(6, 24);

    imuService.begin();

    imuChar.setProperties(CHR_PROPS_NOTIFY);
    imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    imuChar.setMaxLen(BATCH_BYTES);
    imuChar.begin();

    syncChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
    syncChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    syncChar.setMaxLen(sizeof(SyncResponse));
    syncChar.setWriteCallback(sync_write_cb);
    syncChar.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(imuService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

bool transport_ble_active() {
    return g_connected && current_batch_target() > 0;
}

void transport_ble_submit_sample(const ImuPacket& packet) {
    if (!g_connected) {
        return;
    }

    uint8_t batch_target = current_batch_target();
    if (batch_target == 0) {
        return;
    }

    g_batch[g_batch_count] = packet;
    g_batch_count++;

    if (g_batch_count >= batch_target) {
        imuChar.notify(
            reinterpret_cast<uint8_t*>(g_batch),
            g_batch_count * sizeof(ImuPacket)
        );
        g_batch_count = 0;
    }
}
