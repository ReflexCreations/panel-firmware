#include "ledtests.h"
#include "uart.h"
#include "commands.h"
#include "led.h"

static uint8_t input_data[64];

static void test_hardcoded_LEDs();
static void test_solid_color_LEDs();
static void test_segment_solid_color_LEDs();
static void test_commit_leds();

static uint8_t g_offset = 0;
static uint8_t r_offset = 1;
static uint8_t b_offset = 2;

static inline void handle_received_command(Commands cmd) {
    switch (cmd) {
        case Command_Test_Hardcoded_LEDs:
            test_hardcoded_LEDs();
            break;

        case Command_Test_Solid_Color_LEDs:
            uart_expect_data(input_data, 3);
            break;

        case Command_Test_Segment_Solid_Color_LEDs:
            uart_expect_data(input_data, 4);
            break;

        case Command_Test_Commit_LEDs:
            test_commit_leds();
            break;
    }
}

static inline void handle_received_data(Commands cmd) {
    switch (cmd) {
        case Command_Test_Solid_Color_LEDs:
            test_solid_color_LEDs();
            break;

        case Command_Test_Segment_Solid_Color_LEDs:
            test_segment_solid_color_LEDs();
            break;
    }
}

void ledtests_handle_tests() {
    switch (uart_status()) {
        case Status_Received_Command:
            handle_received_command(uart_current_command());
            break;

        case Status_Received_Data:
            handle_received_data(uart_current_command());
            break;
    }
}

static void test_hardcoded_LEDs() {
    uint8_t segment_buffer[64];

    for (uint8_t led = 0; led < 21; led++) {
        segment_buffer[1 + led * 3 + r_offset] = led % 3 == 0 ? 0x20 : 0x00;
        segment_buffer[1 + led * 3 + g_offset] = led % 3 == 1 ? 0x20 : 0x00;
        segment_buffer[1 + led * 3 + b_offset] = led % 3 == 2 ? 0x20 : 0x00;
    } 

    for (uint8_t segment = 0; segment < 4; segment++) {
        segment_buffer[0] = (segment << 4) & 0x30;
        led_prepare_input(segment_buffer);
    }

    led_send_buffer();
}

static void test_solid_color_LEDs() {
    uint8_t segment_buffer[64];

    for (uint8_t led = 0; led < 21; led++) {
        segment_buffer[1 + led * 3 + r_offset] = input_data[0];
        segment_buffer[1 + led * 3 + g_offset] = input_data[1];
        segment_buffer[1 + led * 3 + b_offset] = input_data[2];
    } 

    for (uint8_t segment = 0; segment < 4; segment++) {
        segment_buffer[0] = (segment << 4) & 0x30;
        led_prepare_input(segment_buffer);
    }

    // Remove the actual send here?
    led_send_buffer();
}

static void test_segment_solid_color_LEDs() {
    uint8_t segment_buffer[64];

    uint8_t segment = input_data[0];
    uint8_t red = input_data[1];
    uint8_t green = input_data[2];
    uint8_t blue = input_data[3];

    for (uint8_t led = 0; led < 21; led++) {
        segment_buffer[1 + led * 3 + r_offset] = red;
        segment_buffer[1 + led * 3 + g_offset] = green;
        segment_buffer[1 + led * 3 + b_offset] = blue;
    }

    segment_buffer[0] = (segment << 4) & 0x30;
    led_prepare_input(segment_buffer);
}

static void test_commit_leds() {
    led_send_buffer();
}