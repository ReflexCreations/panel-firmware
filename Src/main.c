#include "main.h"
#include "adc.h"
#include "led.h"
#include "uart.h"
#include "stdbool.h"
#include "debug_leds.h"
#include "commands.h"
#include "commtests.h"
#include "ledtests.h"

static void init(void);
static void test(void);
static void run(void);

static void init_system_clock(void);
static void init_gpio(void);

static uint8_t sensor_out_buffer[8];
static uint8_t LED_segment_in_buffer[64];

int main(void) {
    init();
    //test();
    run();
}

static inline void request_sensors() {
    DBG_LED2_ON();
    adc_read_into(sensor_out_buffer);
    uart_send_response(sensor_out_buffer, 8);
}

static inline void process_LED_segment() {
    uart_expect_data(LED_segment_in_buffer, 64);
}

static inline void process_LED_segment_data_received() {
    led_prepare_input(LED_segment_in_buffer);
}

static inline void commit_LEDs() {
    led_send_buffer();
}

static inline void handle_command_received(Commands cmd) {
    switch (cmd) {
        case Command_Request_Sensors:
            request_sensors();
            break;

        case Command_Process_LED_Segment:
            process_LED_segment();
            break;

        case Command_Commit_LEDs:
            commit_LEDs();
            break;
    }
}

static inline void handle_data_received(Commands cmd) {
    switch (cmd) {
        case Command_Process_LED_Segment:
            process_LED_segment_data_received();
            break;
    }
}

// Main firmware entrypoint
static void run() {
    while (1) {
        switch (uart_status()) {
            case Status_Received_Command:
                handle_command_received(uart_current_command());
                break;

            case Status_Received_Data:
                handle_data_received(uart_current_command());
                break;
        }

        uart_advance();
    }
}

// test entrypoint, run instead of `run()` to test systems.
static void test() {
    while (1) {
        commtests_handle_tests();
        ledtests_handle_tests();
        uart_advance();
    }
}

static void init() {
    HAL_Init();
    init_gpio();
    init_system_clock();

    uart_init();
    adc_init();
    led_init();

    DBG_LED1_ON();
}

static void init_system_clock(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                                |RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK){
        Error_Handler();
    }
}

static void init_gpio(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // A 4,5,6,7: Spare GPIO pins connected to header
    // A 11,12,15: Output enables for RS485 transceiver IC 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

    // B 0,1,3: Status LEDS
    // B 4,5,6,7: RGB LEDs, each segment data line
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                            |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                            |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                            |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void) {
    while(1) {
        DBG_LED2_TOGGLE();
        HAL_Delay(300);
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line){
}
#endif