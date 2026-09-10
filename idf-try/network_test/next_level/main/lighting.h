#pragma once

#include "app_types.h"

void send_update_ack(uint32_t message_id);

void initialize_neopixels();
void update_neopixels(const ThingSpeakData *data);
void led_update_task(void *parameter);
