#ifndef __COMMAND_H
#define __COMMAND_H

typedef enum {
  CMD_NONE = 0x00,
  CMD_REQUEST_SENSORS = 0x01,
  CMD_TRANSMIT_LED_DATA = 0x02,
  CMD_COMMIT_LEDS = 0x03
} Command;

#endif
