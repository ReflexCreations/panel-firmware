#include "commtests.h"
#include "uart.h"
#include "commands.h"
#include "debug_leds.h"

static uint8_t send_data[64];
static uint8_t receive_data[64];

static void expect_2b_cmd_received();
static void expect_64b_cmd_received();
static void double_vals_cmd_received();
static void double_vals_data_received();

static inline void handle_command_received() {
    switch (uart_current_command()) {
        case Command_Test_Expect_2B:
            expect_2b_cmd_received();
            break;

        case Command_Test_Expect_64B:
            expect_64b_cmd_received();
            break;

        case Command_Test_Double_Values:
            double_vals_cmd_received();
            break;

        default: return;
    }
}

static inline void handle_data_received() {
    switch (uart_current_command()) {
        case Command_Test_Double_Values:
            double_vals_data_received();
            break;

        default: return;
    }
}

void commtests_handle_tests() {
    switch (uart_status()) {
        case Status_Received_Command:
            handle_command_received();
            break;

        case Status_Received_Data:
            handle_data_received();
            break;

        default: break;
    }
}

static void expect_2b_cmd_received() {
    DBG_LED3_TOGGLE();
    send_data[0] = 0xBE;
    send_data[1] = 0xEF;
    uart_send_response(send_data, 2);
}

static void expect_64b_cmd_received() {
    for (uint8_t i = 0; i < 64; i ++) {
        send_data[i] = i + 1;
    }

    uart_send_response(send_data, 64);
}

static void double_vals_cmd_received() {
    uart_expect_data(receive_data, 64);
}

static void double_vals_data_received() {
    for (uint8_t i = 0; i < 64; i++) {
        send_data[i] = receive_data[i] * 2;    
    }

    uart_send_response(send_data, 64);
}