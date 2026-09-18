/*
 * Xiao nRF52840 Sense Plus — IMU streamer
 *
 * Streams the same samples over BLE and USB with a shared sequence counter.
 */

#include <Arduino.h>

#include "imu_sampler.h"
#include "protocol.h"
#include "transport_ble.h"
#include "transport_usb.h"

static void blink_error_forever() {
    pinMode(LED_RED, OUTPUT);
    while (true) {
        digitalToggle(LED_RED);
        delay(200);
    }
}

static bool any_transport_ready() {
    if (transport_ble_active()) {
        return true;
    }
    if (transport_usb_ready()) {
        return true;
    }
    return false;
}

static void submit_sample(const ImuPacket& packet) {
    transport_ble_submit_sample(packet);
    transport_usb_submit_sample(packet);
}

void setup() {
    transport_usb_init();

    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);

    if (!imu_sampler_init()) {
        blink_error_forever();
    }

    transport_ble_init();
}

void loop() {
    transport_usb_poll();

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
