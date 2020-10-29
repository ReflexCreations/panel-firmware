#include "uart.h"
#include "debug_leds.h"

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

void Error_Handler();

static void init_gpio();
static void init_periph();
static void init_dma();

static uint8_t ack_msg = MSG_ACKNOWLEGE;

static inline void setStatus(PortStatus newStatus) {
    port_state.status = newStatus;
}

// Returns true if any interrupt flags were present
static inline uint8_t process_interrupt_flags() {
    if (port_state.flag_rx_complete) {
        port_state.flag_rx_complete = false;

        switch (port_state.status) {
            case Status_Receiving_Command:
                setStatus(Status_Received_Command);
                break;

            case Status_Receiving_Data:
                setStatus(Status_Received_Data);
                break;
        }

        return true;
    }

    if (port_state.flag_tx_complete) {
        port_state.flag_tx_complete = false;
        
        if (port_state.status != Status_Receiving_Data) {
            setStatus(Status_Idle);
            return true;
        }
    }

    return false;
}

static inline void send_acknowledge() {
    HAL_UART_Transmit_DMA(&huart1, &ack_msg, 1);
}

void uart_init() {
    init_gpio();
    init_dma();
    init_periph();

    setStatus(Status_Idle);

    // UART, CK - configures it as an output, but irrelevant;
    // using USART periph as UART, clock pin disregarded.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void uart_advance() {
    if (port_state.status == Status_Halted) return;
    if (process_interrupt_flags()) return;

    switch (port_state.status) {
        case Status_Idle:
            port_state.send_data = NULL;
            port_state.send_length = 0;
            port_state.receive_data = NULL;
            port_state.receive_length = 0;

            setStatus(Status_Receiving_Command);
            HAL_UART_Receive_DMA(&huart1, &port_state.current_command, 1);
            break;

        // Waiting.
        case Status_Receiving_Command: break;

        case Status_Received_Command:
            // If we are expecting more data to come in, go to receiving mode
            if (port_state.receive_length > 0) {
                setStatus(Status_Receiving_Data);

                // Note: This assumes that we're setting up "receive" quick
                // enough, as the IO board will just start sending data very
                // soon after the command.
                // Should there ever be an issue here where we're not picking
                // up data after the command, perhaps the system should be
                // changed to include an acknowledge message being sent back
                // from the panel, at which time it immediately begins
                // receiving.
                HAL_UART_Receive_DMA(
                    &huart1,
                    port_state.receive_data,
                    port_state.receive_length
                );

                send_acknowledge();

            } else if (port_state.send_length > 0) {
                // Alternatively, if we're expected to send data, go to sending mode
                // Note: only if we're not also expected to receive more data first            
                setStatus(Status_Sending_Response);
                HAL_UART_Transmit_DMA(
                    &huart1,
                    port_state.send_data,
                    port_state.send_length
                );
            } else {
                // If neither of the two apply, done handle command, back to idle.
                setStatus(Status_Idle);
                send_acknowledge();
            }

            break;

        // Waiting.
        case Status_Receiving_Data: break;

        case Status_Received_Data:
            // If we're expecting to send some data back now we've finished
            // receiving data, go to sending mode.
            if (port_state.send_length > 0) {
                setStatus(Status_Sending_Response);
                HAL_UART_Transmit_DMA(
                    &huart1,
                    port_state.send_data,
                    port_state.send_length
                );

                break;
            }

            // Otherwise, go back to Idle.
            setStatus(Status_Idle);
            send_acknowledge();
            break;

        // Waiting.
        case Status_Sending_Response: break;
    }
}

PortStatus uart_status() {
    return port_state.status;
}

void uart_stop() { setStatus(Status_Halted); }

void uart_start() { setStatus(Status_Idle); }

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
    huart1.Init.BaudRate = 4500000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
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

void USART1_IRQHandler() { HAL_UART_IRQHandler(&huart1); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { 
    port_state.flag_tx_complete = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    port_state.flag_rx_complete = true;
}