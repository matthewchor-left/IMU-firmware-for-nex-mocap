#pragma once

#include <cstdint>

#include "protocol.h"

bool imu_sampler_init();
bool imu_sampler_tick(uint32_t now_us, ImuPacket* packet);
