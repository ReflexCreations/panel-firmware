#ifndef __UART_H
#define __UART_H

#include "bool.h"
#include "commands.h"
#include "stm32f3xx.h"
#include "debug_leds.h"

#define MSG_ACKNOWLEGE (0xACU)

typedef enum {
    Status_Idle = 0B000,
    Status_Receiving_Command = 0B0001,
    Status_Received_Command = 0B0010,
    Status_Receiving_Data = 0B0011,
    Status_Received_Data = 0B0100,
    Status_Sending_Response = 0B0101,
    Status_Halted = 0B0110
} PortStatus;

typedef struct {
    PortStatus status;
    Commands current_command;
    uint8_t *send_data;
    uint8_t send_length;
    uint8_t *receive_data;
    uint8_t receive_length;
    uint8_t flag_rx_complete;
    uint8_t flag_tx_complete;
} PortState;

PortState port_state;

UART_HandleTypeDef huart1;

void uart_init(void);
void uart_advance();
PortStatus uart_status(void);
void uart_start();
void uart_stop();

inline Commands uart_current_command() {
    return port_state.current_command;
}

inline void uart_expect_data(uint8_t *dest_ptr, uint16_t len) {
    port_state.receive_data = dest_ptr;
    port_state.receive_length = len;
}

inline void uart_send_response(uint8_t * source_ptr, uint16_t len) {
    port_state.send_data = source_ptr;
    port_state.send_length = len;
}

#endif
