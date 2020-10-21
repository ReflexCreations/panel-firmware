#ifndef __UART_H
#define __UART_H

#include "bool.h"
#include "command.h"
#include "stm32f3xx.h"
#include "debug_leds.h"

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
  uint8_t raw_command;
  Command current_command;
  uint8_t sensor_data[8];
  uint8_t led_packet[64];
  bool have_led_packet;
  bool should_commit_leds;
} PortState;

PortState port_state;

UART_HandleTypeDef huart1;

void uart_init(void);
void uart_advance();
void uart_take_led_packet(uint8_t *);
void uart_start();
void uart_stop();

#endif
