#include "led.h"
#include "adc.h"
#include "main.h"
#include "stdbool.h"
#include "debug_leds.h"

/**
    A potential performance improvement, if needed:

    Keep two "led_data" arrays: one for the current one being clocked out,
    and another one to process data into when staging data.
    Alternate them on completion of the LED transmission.

    This way, we could do the expensive processing upon receipt of one segment's
    worth of LED data, instead of all at once right before actually committing
    LEDs.

 */

#define LED_PORT GPIOB
#define LED_PINS GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7

#define NUM_LEDS 21
#define NUM_SEGS 4
#define NUM_BYTES_PER_LED 3
#define NUM_BYTES_PER_SEGMENT (NUM_BYTES_PER_LED * NUM_LEDS)

#define LEN_BUFFER (NUM_LEDS * 24)

// Sets the timing for data clocked out to LEDs
// PERIOD sets the overall, well, period; this is clock speed / PERIOD
// CC2_VAL sets the time at which we go back to 0 in case of a "0" bit
// CC3_VAL sets the time at which we go back to 0 regardless of "0" or "1" bit.
#define LED_TIMER_PERIOD  90
#define LED_TIMER_CC2_VAL 16
#define LED_TIMER_CC3_VAL 60

#define NOP asm("nop")

// Uses DMA Channels 2, 3, 6

uint8_t led_pin_pos = 0b11110000;

uint16_t led_pin_masks[4] = {
    ~(0b000000000010000), // Upper left
    ~(0b000000001000000), // Upper right
    ~(0b000000010000000), // Lower left
    ~(0b000000000100000)  // Lower right
};

// Storage for incoming LED data
uint8_t led_staged_data[NUM_SEGS * NUM_BYTES_PER_SEGMENT];

// Processed LED data, ready to be clocked out using DMA
uint16_t led_data[LEN_BUFFER];

volatile bool sending_buffer = false;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef dma_ch2_tim_ch1;
DMA_HandleTypeDef dma_ch3_tim_ch2;
DMA_HandleTypeDef dma_ch6_tim_ch3;

static void init_led_gpio();
static void init_timer_periph();
static void init_timer_events();
static void init_timer_dma();
static void transfer_complete_handler(DMA_HandleTypeDef *dma_handle);

// "Public" functions
void led_init(){
    init_led_gpio();
    init_timer_dma();
    init_timer_periph();
    init_timer_events();

    // Send the blank buffer on init just to turn LEDs off
    led_send_buffer();
}

// Copy one segment's buffer into led_staged_data at the correct offset
void led_stage_buffer(uint8_t *led_buffer) {
    // Get segment number from the first byte of the buffer
    uint8_t segment = (led_buffer[0] >> 4) & 0x03;
    
    // Get pointer to area in array for this segment
    uint8_t * staged_buffer = led_staged_data + segment * NUM_BYTES_PER_SEGMENT;

    for (uint8_t i = 0; i < NUM_BYTES_PER_SEGMENT; i++) {
        staged_buffer[i] = led_buffer[i + 1]; // don't need the header, so + 1
    }
}

static inline void led_prepare_segments() {
    // Reset led_data first
    for (uint16_t address = 0; address < LEN_BUFFER; address++) {
        // Only affect bits of the pins that are relevant to LEDs
        led_data[address] |= led_pin_pos;
    }

    // Iterate over staged data segments, prepare data for clocking out over
    // GPIO lines
    for (uint8_t seg = 0; seg < NUM_SEGS; seg++) {
        // Each segment's data is kept in led_staged_data sequentially
        // Get a pointer to where in that array this segment's data begins 
        uint8_t * buffer = led_staged_data + (NUM_BYTES_PER_SEGMENT * seg);
        uint16_t seg_mask = led_pin_masks[seg];

        for (uint8_t l_byte = 0; l_byte < NUM_BYTES_PER_SEGMENT; l_byte++) {
            uint8_t buf_byte = buffer[l_byte];
            uint16_t * lane_data = led_data + l_byte * 8;

            if (buf_byte & 0b10000000) lane_data[0] &= seg_mask;
            if (buf_byte & 0b01000000) lane_data[1] &= seg_mask;
            if (buf_byte & 0b00100000) lane_data[2] &= seg_mask;
            if (buf_byte & 0b00010000) lane_data[3] &= seg_mask;
            if (buf_byte & 0b00001000) lane_data[4] &= seg_mask;
            if (buf_byte & 0b00000100) lane_data[5] &= seg_mask;
            if (buf_byte & 0b00000010) lane_data[6] &= seg_mask;
            if (buf_byte & 0b00000001) lane_data[7] &= seg_mask;
        }
    }
}

void led_send_buffer() {
    if (sending_buffer) return;

    DBG_LED3_ON();

    // Process staged data into data to be clocked out on the GPIO port
    led_prepare_segments();

    sending_buffer = true;

	__HAL_DMA_CLEAR_FLAG(&dma_ch2_tim_ch1, DMA_FLAG_TC2 | DMA_FLAG_HT2 | DMA_FLAG_TE2);
	__HAL_DMA_CLEAR_FLAG(&dma_ch3_tim_ch2, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3);
	__HAL_DMA_CLEAR_FLAG(&dma_ch6_tim_ch3, DMA_FLAG_TC6 | DMA_FLAG_HT6 | DMA_FLAG_TE6);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3);

    //init_timer_dma();
    //init_timer_periph();
    //init_timer_events();

    // Set all LED data pins high on timer update event
    HAL_DMA_Start_IT(
        &dma_ch2_tim_ch1,
        (uint32_t) &led_pin_pos,
        (uint32_t) &LED_PORT->BSRR,
        LEN_BUFFER
    );

    // Resets select LED data lines at one third through timer cycle, that is
    // if those data lines are meant to output a "0" to the LED at that time.
	HAL_DMA_Start_IT(
        &dma_ch3_tim_ch2,
        (uint32_t) led_data,
        (uint32_t) &LED_PORT->BRR,
        LEN_BUFFER
    );

    // Reset all LED data lines at two thirds* through timer cycle
    // * well, see datasheet for exact timing
    HAL_DMA_Start_IT(
        &dma_ch6_tim_ch3,
        (uint32_t) &led_pin_pos,
        (uint32_t) &LED_PORT->BRR,
        LEN_BUFFER
    );

     __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3);

    // Very important to ensure LEDs are synchronised right.
    // Without this there can be an off-by-one error that causes a flickery
    // mess on the LEDs
    TIM1->CNT = LED_TIMER_PERIOD - 1;
	__HAL_TIM_ENABLE(&htim1);
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

void TIM1_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim1);
}

static void init_timer_periph(){
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Period = LED_TIMER_PERIOD;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.Prescaler = 0;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

    HAL_TIM_PWM_Init(&htim1);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

static void init_timer_events(){
    TIM_OC_InitTypeDef htimoc;

    uint32_t channel_1_event_time = 1;
    uint32_t channel_2_event_time = LED_TIMER_CC2_VAL;
    uint32_t channel_3_event_time = LED_TIMER_CC3_VAL;

    htimoc.OCMode = TIM_OCMODE_ACTIVE;
    htimoc.OCPolarity = TIM_OCPOLARITY_HIGH;
    htimoc.OCFastMode = TIM_OCFAST_DISABLE;
    htimoc.OCIdleState = TIM_OCIDLESTATE_RESET;
    htimoc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    htimoc.Pulse = channel_1_event_time;
    HAL_TIM_PWM_ConfigChannel(&htim1, &htimoc, TIM_CHANNEL_1);

    htimoc.Pulse = channel_2_event_time;
    HAL_TIM_PWM_ConfigChannel(&htim1, &htimoc, TIM_CHANNEL_2);

    htimoc.Pulse = channel_3_event_time;
    HAL_TIM_PWM_ConfigChannel(&htim1, &htimoc, TIM_CHANNEL_3);

    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; 

    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

static void init_timer_dma(){
    __HAL_RCC_DMA1_CLK_ENABLE();

    dma_ch2_tim_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_ch2_tim_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_ch2_tim_ch1.Init.MemInc = DMA_MINC_DISABLE;
    dma_ch2_tim_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_ch2_tim_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_ch2_tim_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_ch2_tim_ch1.Init.Mode = DMA_NORMAL;
    dma_ch2_tim_ch1.Instance = DMA1_Channel2;
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], dma_ch2_tim_ch1);
    HAL_DMA_Init(&dma_ch2_tim_ch1);

    dma_ch3_tim_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_ch3_tim_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_ch3_tim_ch2.Init.MemInc = DMA_MINC_ENABLE;
    dma_ch3_tim_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_ch3_tim_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_ch3_tim_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_ch3_tim_ch2.Init.Mode = DMA_NORMAL;
    dma_ch3_tim_ch2.Instance = DMA1_Channel3;
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC2], dma_ch3_tim_ch2);
    HAL_DMA_Init(&dma_ch3_tim_ch2);

    dma_ch6_tim_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_ch6_tim_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_ch6_tim_ch3.Init.MemInc = DMA_MINC_DISABLE;
    dma_ch6_tim_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_ch6_tim_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_ch6_tim_ch3.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_ch6_tim_ch3.Init.Mode = DMA_NORMAL;
    dma_ch6_tim_ch3.Instance = DMA1_Channel6;
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC3], dma_ch6_tim_ch3);
    HAL_DMA_Init(&dma_ch6_tim_ch3);

    dma_ch2_tim_ch1.XferCpltCallback = transfer_complete_handler;
    dma_ch3_tim_ch2.XferCpltCallback = transfer_complete_handler;
    dma_ch6_tim_ch3.XferCpltCallback = transfer_complete_handler;

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma_ch2_tim_ch1);
}

void DMA1_Channel3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma_ch3_tim_ch2);
}

void DMA1_Channel6_IRQHandler(void){
    HAL_DMA_IRQHandler(&dma_ch6_tim_ch3);
}

static void transfer_complete_handler(DMA_HandleTypeDef *dma_handle){
    dma_handle->Instance->CCR &= ~DMA_CCR_EN;

    if (dma_handle == &dma_ch2_tim_ch1) {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC1);
    } else if (dma_handle == &dma_ch3_tim_ch2) {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC2);
    } else if (dma_handle == &dma_ch6_tim_ch3) {
        __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);
    }

    if (dma_handle != &dma_ch2_tim_ch1) return;
    __HAL_TIM_DISABLE(&htim1);
    TIM1->CR1 = 0;
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3);

    LED_PORT->BRR |= led_pin_pos;

    // Clear LED data
    // This is also done elsewhere, but leaving it here reduces LED flickering
    // too.
    /*for (uint16_t address = 0; address < LEN_BUFFER; address++){
        led_data[address] |= led_pin_pos;
    }*/

    sending_buffer = false;

    DBG_LED3_OFF();
}