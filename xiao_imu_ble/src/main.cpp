/*
 * Xiao nRF52840 Sense Plus — IMU streamer
 *
 * Build with TRANSPORT_BLE (default) and/or TRANSPORT_USB to select outputs.
 * Both transports share the same sampler, sequence counter, and 29-byte sample layout.
 */

#include <Arduino.h>

#include "imu_sampler.h"
#include "protocol.h"

#if defined(TRANSPORT_BLE)
#include "transport_ble.h"
#endif

#if defined(TRANSPORT_USB)
#include "transport_usb.h"
#endif

#ifndef TRANSPORT_BLE
#ifndef TRANSPORT_USB
#define TRANSPORT_BLE 1
#endif
#endif

static void blink_error_forever() {
    pinMode(LED_RED, OUTPUT);
    while (true) {
        digitalToggle(LED_RED);
        delay(200);
    }
}

static bool any_transport_ready() {
#if defined(TRANSPORT_BLE)
    if (transport_ble_active()) {
        return true;
    }
#endif
#if defined(TRANSPORT_USB)
    if (transport_usb_ready()) {
        return true;
    }
#endif
    return false;
}

static void submit_sample(const ImuPacket& packet) {
#if defined(TRANSPORT_BLE)
    transport_ble_submit_sample(packet);
#endif
#if defined(TRANSPORT_USB)
    transport_usb_submit_sample(packet);
#endif
}

void setup() {
#if defined(TRANSPORT_USB)
    transport_usb_init();
#elif defined(TRANSPORT_BLE)
    Serial.begin(115200);
    uint32_t start = millis();
    while (!Serial && millis() - start < 2000) {
        delay(100);
    }
#endif

    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);

    if (!imu_sampler_init()) {
#if defined(TRANSPORT_BLE) && !defined(TRANSPORT_USB)
        Serial.println("ERROR: IMU init failed");
#endif
        blink_error_forever();
    }

#if defined(TRANSPORT_BLE)
    transport_ble_init();
#if defined(TRANSPORT_BLE) && !defined(TRANSPORT_USB)
    Serial.println("XiaoIMU BLE streamer ready");
#endif
#endif
}

void loop() {
#if defined(TRANSPORT_USB)
    transport_usb_poll();
#endif

    const uint32_t now = micros();
    ImuPacket packet;

    if (!imu_sampler_tick(now, &packet)) {
        return;
    }

    if (!any_transport_ready()) {
        return;
    }

    submit_sample(packet);
}
