#include "imu_sampler.h"

#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>

static constexpr uint8_t REG_OUTX_L_G = 0x22;
static constexpr uint8_t BULK_READ_LEN = 12;

static LSM6DS3 imu(I2C_MODE, 0x6A);
static uint32_t g_last_sample_us = 0;
static uint8_t g_sequence = 0;

// Gyro: raw * 4.375 * (range/125) / 1000 => dps  (range=1000)
static const float gyroScale = 4.375f * (1000.0f / 125.0f) / 1000.0f;
// Accel: raw * 0.061 * (range>>1) / 1000 => g     (range=8)
static const float accelScale = 0.061f * (8 >> 1) / 1000.0f;

static int16_t to_i16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>(lo | (hi << 8));
}

bool imu_sampler_init() {
    imu.settings.gyroEnabled = 1;
    imu.settings.gyroRange = 1000;
    imu.settings.gyroSampleRate = 208;
    imu.settings.accelEnabled = 1;
    imu.settings.accelRange = 8;
    imu.settings.accelSampleRate = 208;

    if (imu.begin() != 0) {
        return false;
    }

    g_last_sample_us = micros();
    return true;
}

bool imu_sampler_tick(uint32_t now_us, ImuPacket* packet) {
    if (packet == nullptr) {
        return false;
    }

    if (now_us - g_last_sample_us < SAMPLE_INTERVAL_US) {
        return false;
    }
    g_last_sample_us += SAMPLE_INTERVAL_US;

    uint8_t buf[BULK_READ_LEN];
    if (imu.readRegisterRegion(buf, REG_OUTX_L_G, BULK_READ_LEN) != 0) {
        return false;
    }

    packet->timestamp_us = now_us;
    packet->sequence = g_sequence++;
    packet->gx = to_i16(buf[0], buf[1]) * gyroScale;
    packet->gy = to_i16(buf[2], buf[3]) * gyroScale;
    packet->gz = to_i16(buf[4], buf[5]) * gyroScale;
    packet->ax = to_i16(buf[6], buf[7]) * accelScale * G_TO_MS2;
    packet->ay = to_i16(buf[8], buf[9]) * accelScale * G_TO_MS2;
    packet->az = to_i16(buf[10], buf[11]) * accelScale * G_TO_MS2;
    return true;
}
