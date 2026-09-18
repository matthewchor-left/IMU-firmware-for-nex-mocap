#include "transport_usb.h"

#include <Arduino.h>
#include <string.h>

static ImuPacket g_batch[USB_BATCH_CAPACITY];
static uint8_t g_batch_count = 0;

static uint8_t g_rx_buffer[128];
static size_t g_rx_len = 0;

static void send_usb_frame(uint8_t msg_type, const uint8_t* payload, uint16_t payload_len) {
    const uint8_t header[USB_HEADER_SIZE] = {
        USB_MAGIC[0],
        USB_MAGIC[1],
        USB_MAGIC[2],
        USB_MAGIC[3],
        USB_FRAME_VERSION,
        msg_type,
        static_cast<uint8_t>(payload_len & 0xFF),
        static_cast<uint8_t>((payload_len >> 8) & 0xFF),
    };

    Serial.write(header, USB_HEADER_SIZE);
    if (payload_len > 0 && payload != nullptr) {
        Serial.write(payload, payload_len);
    }
}

static void flush_batch() {
    if (g_batch_count == 0) {
        return;
    }

    const uint16_t payload_len = static_cast<uint16_t>(g_batch_count * IMU_PACKET_SIZE);
    send_usb_frame(
        USB_MSG_IMU_BATCH,
        reinterpret_cast<uint8_t*>(g_batch),
        payload_len
    );
    g_batch_count = 0;
}

static void handle_sync_request(const uint8_t* payload, uint16_t len) {
    if (len != USB_SYNC_REQUEST_SIZE) {
        return;
    }

    const uint32_t received_at_us = micros();

    SyncResponse response;
    memcpy(&response.request_id, payload, sizeof(response.request_id));
    response.timestamp_us = received_at_us;
    send_usb_frame(
        USB_MSG_SYNC_RESP,
        reinterpret_cast<uint8_t*>(&response),
        sizeof(response)
    );
}

static void consume_rx_buffer() {
    while (g_rx_len >= USB_HEADER_SIZE) {
        if (g_rx_buffer[0] != USB_MAGIC[0] ||
            g_rx_buffer[1] != USB_MAGIC[1] ||
            g_rx_buffer[2] != USB_MAGIC[2] ||
            g_rx_buffer[3] != USB_MAGIC[3]) {
            memmove(g_rx_buffer, g_rx_buffer + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }

        const uint8_t msg_type = g_rx_buffer[5];
        const uint16_t payload_len =
            static_cast<uint16_t>(g_rx_buffer[6]) |
            (static_cast<uint16_t>(g_rx_buffer[7]) << 8);
        const size_t frame_len = USB_HEADER_SIZE + payload_len;
        if (g_rx_len < frame_len) {
            return;
        }

        const uint8_t* payload = g_rx_buffer + USB_HEADER_SIZE;
        if (msg_type == USB_MSG_SYNC_REQ) {
            handle_sync_request(payload, payload_len);
        }

        memmove(g_rx_buffer, g_rx_buffer + frame_len, g_rx_len - frame_len);
        g_rx_len -= frame_len;
    }
}

void transport_usb_init() {
    Serial.begin(115200);
    g_batch_count = 0;
    g_rx_len = 0;
}

bool transport_usb_ready() {
    return static_cast<bool>(Serial);
}

void transport_usb_poll() {
    while (Serial.available() > 0) {
        const int byte = Serial.read();
        if (byte < 0) {
            break;
        }
        if (g_rx_len >= sizeof(g_rx_buffer)) {
            g_rx_len = 0;
        }
        g_rx_buffer[g_rx_len++] = static_cast<uint8_t>(byte);
    }
    consume_rx_buffer();
}

void transport_usb_submit_sample(const ImuPacket& packet) {
    if (!transport_usb_ready()) {
        return;
    }

    g_batch[g_batch_count] = packet;
    g_batch_count++;

    if (g_batch_count >= USB_BATCH_CAPACITY) {
        flush_batch();
    }
}
