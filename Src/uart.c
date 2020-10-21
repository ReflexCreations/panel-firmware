#include "uart.h"
#include "command.h"
#include "debug_leds.h"


DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;


void Error_Handler();

static void init_gpio();
static void init_periph();
static void init_dma();
static void process_command(Command);

static inline void setStatus(PortStatus newStatus) {
    port_state.status = newStatus;
    DBG_EXT_WRITE((uint8_t)newStatus);
}

void uart_init() {
    init_gpio();
    init_dma();
    init_periph();

    setStatus(Status_Idle);
    port_state.have_led_packet = false;
    port_state.should_commit_leds = false;

    // UART, CK - configures it as an output, but irrelevant;
    // using USART periph as UART, clock pin disregarded.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void uart_stop() {
    //setStatus(Status_Halted);
}

void uart_start() {
    //setStatus(Status_Idle);
}

void uart_take_led_packet(uint8_t * target) {
    if (!port_state.have_led_packet) return;
    port_state.have_led_packet = false;

    for (uint8_t i = 0; i < 64; i++) {
        target[i] = port_state.led_packet[i];
    }
}

static void init_gpio() {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Toggle RS-485 tranceivers 
    // UART, RX_EN - configures it as input
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    // UART, TX_EN - configures it as output
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    // UART, CK_EN - configures it as output
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void init_periph() {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 1984000;//512000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8; // 16
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE; // DIS
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    
    HAL_NVIC_EnableIRQ(USART1_IRQn);

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

// No idea what I ought to name this... advance the state machine
void uart_advance() {
    switch (port_state.status) {
        case Status_Idle:
            HAL_UART_Receive_DMA(&huart1, &port_state.raw_command, 1);
            setStatus(Status_Receiving_Command);
            break;

        case Status_Receiving_Command:
            break;

        case Status_Received_Command:
            process_command(port_state.current_command);
            break;

        case Status_Receiving_Data:
            break;

        case Status_Received_Data:
            if (port_state.current_command == CMD_TRANSMIT_LED_DATA) {
                port_state.have_led_packet = true;
            }

            setStatus(Status_Idle);
            break;

        case Status_Sending_Response:
            break;

        case Status_Halted:
            break;
    }
}

static void process_command(Command command) {
    switch (command) {
        case CMD_REQUEST_SENSORS:
            setStatus(Status_Sending_Response);
            HAL_UART_Transmit_DMA(&huart1, port_state.sensor_data, 8);
            break;

        case CMD_TRANSMIT_LED_DATA:
            setStatus(Status_Receiving_Data);
            HAL_UART_Receive_DMA(&huart1, port_state.led_packet, 64);
            break;

        case CMD_COMMIT_LEDS:
            setStatus(Status_Idle);
            port_state.should_commit_leds = true;
            break;
    }
}

void USART1_IRQHandler() { HAL_UART_IRQHandler(&huart1); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{	
    //DBG_LED2_OFF();
    setStatus(Status_Idle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    switch (port_state.status) {
        case Status_Receiving_Command:
            port_state.current_command = (port_state.raw_command >> 4) & 0x0F;
            setStatus(Status_Received_Command);
            break;
        case Status_Receiving_Data:
            setStatus(Status_Received_Data);
            break;
    }
}