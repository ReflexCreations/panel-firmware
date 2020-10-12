#include "main.h"
#include "adc.h"
#include "led.h"
#include "uart.h"
#include "bool.h"

#define DBG_LED1_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define DBG_LED1_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define DBG_LED1_TOGGLE() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0)
#define DBG_LED2_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define DBG_LED2_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define DBG_LED2_TOGGLE() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1)
#define DBG_LED3_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define DBG_LED3_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define DBG_LED3_TOGGLE() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3)

typedef enum {
  CMD_REQUEST_SENSORS = 0x01,
  CMD_TRANSMIT_LED_DATA = 0x02,
  CMD_COMMIT_LEDS = 0x03
} command_t;

// uart.c
extern UART_HandleTypeDef huart1;
extern uart_txbuffer_t uart_txbuffer;

// adc.c
extern bool adc_ready;
extern GuardedData output_data;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


int main(void) {
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  uart_init();
  adc_init();
  led_init();

  DBG_LED1_ON();

  uint8_t command_raw;
  uint8_t in_buffer[64];
  output_data.beingRead = false;
  
  while (1) {
    // If there was data to be sent via UART, while it was already still sending,
    // this ensures that data is moved into the array that will be used for DMA
    uart_move_queue();

    HAL_UART_Receive(&huart1, &command_raw, 1, HAL_MAX_DELAY);
    
    command_t command_value = (command_raw >> 4) & 0x0F;

    if (command_value == CMD_REQUEST_SENSORS){
      DBG_LED1_TOGGLE();

      // Split 16 bit sensor data into bytes, for byte-aligned transmission
      output_data.beingRead = 1;

      // Select which buffer we're going to write to based on whether we're currently sending
      // If we are sending, we stick it in queued data, otherwise in transmit_data.
      bool was_sending = uart_txbuffer.sending;
      uint8_t *tx_buffer_arr = was_sending 
        ? uart_txbuffer.queued_data
        : uart_txbuffer.transmit_data;

      for (uint8_t i = 0; i < 4; i++){
        tx_buffer_arr[i * 2 + 0] = output_data.data[i].as_bytes[0];
        tx_buffer_arr[i * 2 + 1] = output_data.data[i].as_bytes[1];
      }
      output_data.beingRead = 0;
      uart_txbuffer.has_queued_data = was_sending;

      // With the queueing system this should only ever, at most, be one
      // reading behind the actual thing. If it's still slow, the queued
      // data is overridden for the next send
      uart_send();

    } else if (command_value == CMD_TRANSMIT_LED_DATA) {

      // TODO: uart_receive stuff to go in uart.c
      HAL_UART_Receive(&huart1, in_buffer, 64, HAL_MAX_DELAY);
      led_prepare_input(in_buffer);

    } else if (command_value == CMD_COMMIT_LEDS) {

      led_send_buffer();

    }
  }
}

void SystemClock_Config(void){
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

static void MX_GPIO_Init(void){
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void){

}

#ifdef  USE_FULL_ASSERT
void assert_failed(char *file, uint32_t line){
}
#endif