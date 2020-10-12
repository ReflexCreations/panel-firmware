#include "uart.h"


DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;


void Error_Handler();

static void init_gpio();
static void init_periph();
static void init_dma();

void uart_init() {
    init_gpio();
    init_periph();

    // UART, CK - configures it as an output, but irrelevant;
    // using USART periph as UART, clock pin disregarded.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    init_dma();
}

static void init_gpio() {
    // Toggle RS-485 tranceivers 
    // UART, RX_EN - configures it as input
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    // UART, TX_EN - configures it as output
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    // UART, CK_EN - configures it as output
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // TODO: move to init_periph?
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void init_periph() {
    HAL_UART_MspInit(&huart1);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 512000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK){
        Error_Handler();
    }
}

static void init_dma() {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}


HAL_StatusTypeDef HAL_UART_DMA_Tx_Stop(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT);
  if((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Abort the UART DMA Tx channel */
    if(huart->hdmatx != NULL)
    {
      HAL_DMA_Abort(huart->hdmatx);
    }
    //UART_EndTxTransfer(huart);
		  /* Disable TXEIE and TCIE interrupts */
		CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
		huart->gState = HAL_UART_STATE_READY;
		
		return HAL_OK;
  }
	
  return HAL_ERROR;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{	
    HAL_UART_DMA_Tx_Stop(&huart1);
    uart_txbuffer.sending = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
}