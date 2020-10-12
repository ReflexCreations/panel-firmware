#ifndef __LED_H
#define __LED_H

#include "stm32f3xx_hal.h"


void DMA1_Channel2_IRQHandler(void);

void led_send_buffer();
void led_init();
void led_prepare_input(uint8_t* led_buffer);
void transfer_complete_handler(DMA_HandleTypeDef *dma_handle);

#endif