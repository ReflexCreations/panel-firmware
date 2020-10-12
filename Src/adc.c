#include "adc.h"

// Uses DMA Channel 1

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint8_t sample_counter = 0;
uint16_t adc_read_buffer[4];
uint32_t accumulated_read_buffer[4];
GuardedData output_data;

bool adc_ready = false;

static void init_adc_periph();
static void init_adc_channels();
static void init_adc_gpio();
static void init_adc_dma();

// Public interface to ADC

void adc_init(){
    init_adc_periph();
    init_adc_gpio();
    init_adc_channels();
    init_adc_dma();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_read_buffer, 4);
}

void adc_start(){
    adc_init();
}

void adc_stop(){
    HAL_DMA_Abort(&hdma_adc1);
    HAL_ADC_DeInit(&hadc1);
}

// "Private" functions
static void init_adc_periph(){
    __HAL_RCC_ADC12_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    HAL_NVIC_SetPriority(ADC1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
    HAL_ADC_Init(&hadc1);
}

static void init_adc_channels(){
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef channel = {0};
    multimode.Mode = ADC_MODE_INDEPENDENT;
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    channel.Channel = ADC_CHANNEL_1;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &channel);
    channel.Channel = ADC_CHANNEL_2;
    channel.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &channel);
    channel.Channel = ADC_CHANNEL_3;
    channel.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &channel);
    channel.Channel = ADC_CHANNEL_4;
    channel.Rank = ADC_REGULAR_RANK_4;
    HAL_ADC_ConfigChannel(&hadc1, &channel);
}

static void init_adc_gpio(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef adc_gpio = {0};
    adc_gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    adc_gpio.Mode = GPIO_MODE_ANALOG;
    adc_gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &adc_gpio);
}

static void init_adc_dma(){
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

// HAL interfacing

void ADC_IRQHandler(){
    HAL_ADC_IRQHandler(&hadc1);
}

void DMA1_Channel1_IRQHandler(){
    HAL_DMA_IRQHandler(&hdma_adc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    uint8_t i = 0;
    if (sample_counter >= 63 && !output_data.beingRead) {
        for (i = 0; i < 4; i++) {
            output_data.data[i].as_word = accumulated_read_buffer[i] / sample_counter;
            accumulated_read_buffer[i] = 0;
        }

        sample_counter = 0;
        adc_ready = true;
    }

    for (i = 0; i < 4; i++) {
        accumulated_read_buffer[i] += adc_read_buffer[i];
    }

    sample_counter += 1;
}
