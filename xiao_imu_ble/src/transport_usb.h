#pragma once

#include "protocol.h"

void transport_usb_init();
void transport_usb_poll();
bool transport_usb_ready();
void transport_usb_submit_sample(const ImuPacket& packet);
