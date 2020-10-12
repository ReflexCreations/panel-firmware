#include "led.h"
#include "adc.h"

#define LED_PORT GPIOB
#define LED_PINS GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
#define NUM_LEDS 21
#define NUM_SEGS 4
#define NUM_PANELS 4
#define LEN_BUFFER NUM_LEDS * 24
#define LEN_TRANSFER LEN_BUFFER / 8

// Uses DMA Channels 2, 3, 6

uint16_t led_pin_pos = 0b11110000;
uint16_t led_pin_positions[4] = {4, 6, 7, 5};
uint16_t *led_pin_ref = &led_pin_pos;
uint32_t timer_period;
uint16_t led_data[LEN_BUFFER];

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef dma_update;
DMA_HandleTypeDef dma_channel_1;
DMA_HandleTypeDef dma_channel_3;

static void init_led_gpio();
static void init_timer_periph();
static void init_timer_events();
static void init_timer_dma();

// "Public" functions
void led_init(){
    init_led_gpio();
    init_timer_dma();
    init_timer_periph();
    init_timer_events();
}

void led_prepare_input(uint8_t *led_buffer){
    uint16_t lane = led_pin_positions[(led_buffer[0] >> 4) & 0x03];
    uint8_t l_byte; 
    for(l_byte = 0; l_byte < 63; l_byte++){
        if(led_buffer[l_byte + 1] & (1 << 7)) led_data[l_byte * 8 + 0] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 6)) led_data[l_byte * 8 + 1] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 5)) led_data[l_byte * 8 + 2] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 4)) led_data[l_byte * 8 + 3] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 3)) led_data[l_byte * 8 + 4] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 2)) led_data[l_byte * 8 + 5] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 1)) led_data[l_byte * 8 + 6] &= ~(1 << lane);
        if(led_buffer[l_byte + 1] & (1 << 0)) led_data[l_byte * 8 + 7] &= ~(1 << lane);
    }
}

void led_send_buffer() {
    // Stop ADC collection while sending LED buffer
    adc_stop();

	HAL_DMA_Start(&dma_update, (uint32_t) led_pin_ref, (uint32_t) &LED_PORT->BSRR, LEN_BUFFER);
	HAL_DMA_Start(&dma_channel_1, (uint32_t) led_data, (uint32_t) &LED_PORT->BRR, LEN_BUFFER);
	HAL_DMA_Start_IT(&dma_channel_3, (uint32_t) led_pin_ref, (uint32_t) &LED_PORT->BRR, LEN_BUFFER);
	__HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE | TIM_DMA_CC1 | TIM_DMA_CC3);
	HAL_TIM_Base_Start(&htim3);
}

// Private functions

static void init_led_gpio(){
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef led_gpio = {0};
    led_gpio.Pin = LED_PINS;
    led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    led_gpio.Pull = GPIO_NOPULL;
    led_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED_PORT, &led_gpio);
}

static void init_timer_periph(){
    __HAL_RCC_TIM3_CLK_ENABLE();

    timer_period = SystemCoreClock / 800000;

    htim3.Instance = TIM3;
    htim3.Init.Period = timer_period;
    htim3.Init.RepetitionCounter = 0;
    htim3.Init.Prescaler = 0;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&htim3);
}

static void init_timer_events(){
    TIM_OC_InitTypeDef htimoc;

    uint32_t channel_1_event_time = 19;
    uint32_t channel_3_event_time = 70;

    htimoc.OCMode = TIM_OCMODE_PWM1;
    htimoc.OCPolarity = TIM_OCPOLARITY_HIGH;
    htimoc.Pulse = channel_1_event_time;
    htimoc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &htimoc, TIM_CHANNEL_1);

    htimoc.Pulse = channel_3_event_time;
    HAL_TIM_PWM_ConfigChannel(&htim3, &htimoc, TIM_CHANNEL_3);
}

static void init_timer_dma(){
    __HAL_RCC_DMA1_CLK_ENABLE();

    dma_update.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_update.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_update.Init.MemInc = DMA_MINC_DISABLE;
    dma_update.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_update.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    dma_update.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_update.Init.Mode = DMA_NORMAL;
    dma_update.Instance = DMA1_Channel3;
    __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_UPDATE], dma_update);
    HAL_DMA_Init(&dma_update);

    dma_channel_1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_channel_1.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_channel_1.Init.MemInc = DMA_MINC_ENABLE;
    dma_channel_1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_channel_1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_channel_1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_channel_1.Init.Mode = DMA_NORMAL;
    dma_channel_1.Instance = DMA1_Channel6;
    __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC1], dma_channel_1);
    HAL_DMA_Init(&dma_channel_1);

    dma_channel_3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_channel_3.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_channel_3.Init.MemInc = DMA_MINC_DISABLE;
    dma_channel_3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_channel_3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    dma_channel_3.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_channel_3.Init.Mode = DMA_NORMAL;
    dma_channel_3.Instance = DMA1_Channel2;
    __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC3], dma_channel_3);
    HAL_DMA_Init(&dma_channel_3);

    dma_channel_3.XferCpltCallback = transfer_complete_handler;
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

void DMA1_Channel2_IRQHandler(void){
    HAL_DMA_IRQHandler(&dma_channel_3);
}

void transfer_complete_handler(DMA_HandleTypeDef *dma_handle){
    HAL_DMA_Abort(&dma_update);
    HAL_DMA_Abort(&dma_channel_1);
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_UPDATE | TIM_DMA_CC1 | TIM_DMA_CC3);
    LED_PORT->BRR |= *led_pin_ref;
    HAL_TIM_Base_Stop(&htim3);

    // Restart the ADC now LED transfer is complete
    adc_start();

    for(uint16_t address = 0; address < LEN_BUFFER; address++){
        led_data[address] = 0xFF;
    }
}


