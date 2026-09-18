#pragma once

#include "protocol.h"

void transport_ble_init();
bool transport_ble_active();
void transport_ble_submit_sample(const ImuPacket& packet);
