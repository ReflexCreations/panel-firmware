#ifndef __LED_H
#define __LED_H

#include "stm32f3xx_hal.h"

void led_init();
void led_stage_buffer(uint8_t * led_buffer);
void led_send_buffer();

#endif