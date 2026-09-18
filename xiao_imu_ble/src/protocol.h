#pragma once

#include <cstddef>
#include <cstdint>

static constexpr uint32_t SAMPLE_INTERVAL_US = 4808; // 208 Hz
static constexpr float G_TO_MS2 = 9.80665f;
static constexpr std::size_t IMU_PACKET_SIZE = 29;

#pragma pack(push, 1)
struct ImuPacket {
    uint32_t timestamp_us;
    uint8_t sequence;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
};
#pragma pack(pop)
static_assert(sizeof(ImuPacket) == IMU_PACKET_SIZE, "ImuPacket must be 29 bytes");

#pragma pack(push, 1)
struct SyncResponse {
    uint32_t request_id;
    uint32_t timestamp_us;
};
#pragma pack(pop)
static_assert(sizeof(SyncResponse) == 8, "SyncResponse must be 8 bytes");

// USB CDC framing shared with the Python proxy.
static constexpr char USB_MAGIC[4] = {'X', 'I', 'M', 'U'};
static constexpr uint8_t USB_FRAME_VERSION = 1;
static constexpr uint8_t USB_MSG_IMU_BATCH = 0x01;
static constexpr uint8_t USB_MSG_SYNC_REQ = 0x02;
static constexpr uint8_t USB_MSG_SYNC_RESP = 0x03;
static constexpr uint8_t USB_HEADER_SIZE = 8;
static constexpr uint8_t USB_SYNC_REQUEST_SIZE = 4;
static constexpr uint8_t USB_BATCH_CAPACITY = 16;
