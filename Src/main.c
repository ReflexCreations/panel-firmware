#include "main.h"
#include "adc.h"
#include "led.h"
#include "uart.h"
#include "bool.h"
#include "debug_leds.h"
#include "command.h"

// uart.c
extern PortState port_state;

// adc.c
extern bool adc_ready;
extern GuardedData output_data;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);


int main(void) {
  HAL_Init();
  MX_GPIO_Init();
  SystemClock_Config();

  uart_init();
  adc_init();
  led_init();

  DBG_LED1_ON();

  uint8_t in_buffer[64];
  output_data.beingRead = false;
  
  while (1) {
    if (port_state.have_led_packet) {
      uart_take_led_packet(in_buffer);
      led_prepare_input(in_buffer);
    }

    if (port_state.should_commit_leds) {
      port_state.should_commit_leds = false;
      led_send_buffer();
    }

    if (port_state.status == Status_Received_Command) {
      switch (port_state.current_command) {
        // Request sensors needs our bit of work here before responding
        case CMD_REQUEST_SENSORS:
          DBG_LED2_ON();
          output_data.beingRead = true;
          for (uint8_t i = 0; i < 4; i++) {
            port_state.sensor_data[i * 2 + 0] = output_data.data[i].as_bytes[0];
            port_state.sensor_data[i * 2 + 1] = output_data.data[i].as_bytes[1];
          }
          output_data.beingRead = false;
          break;

        default: break;
      }
    }

    uart_advance();
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