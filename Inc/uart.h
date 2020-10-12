#ifndef __UART_H
#define __UART_H

#include "bool.h"
#include "stm32f3xx.h"

typedef struct {
    uint8_t transmit_data[8];
    uint8_t queued_data[8];
    bool sending;
    bool has_queued_data;
} uart_txbuffer_t;

uart_txbuffer_t uart_txbuffer;
UART_HandleTypeDef huart1;

void uart_init(void);

inline void uart_send(void) {
    uart_txbuffer.sending = true;
    HAL_UART_Transmit_DMA(&huart1, uart_txbuffer.transmit_data, 8);
}

inline void uart_move_queue(void) {
    if (!uart_txbuffer.has_queued_data || uart_txbuffer.sending) {
      return;
    }

    for (uint8_t i = 0; i < 8; i++) {
      uart_txbuffer.transmit_data[i] = uart_txbuffer.queued_data[i];
    }

    uart_txbuffer.has_queued_data = false;
}

#endif
