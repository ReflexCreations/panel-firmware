#ifndef __ADC_H
#define __ADC_H

#include "stm32f3xx_hal.h"
#include "bool.h"

// Like a 4-element array, but contains a "bool" indicating data is currently
// being read, so that we can prevent writing to it while that's the case

// Allows representing one sensor reading as either a uint16_t or 2-element array of uint8_ts
typedef union {
    uint16_t as_word;
    uint8_t as_bytes[2];
} DualAccessData;

typedef struct {
    DualAccessData data[4];
    bool beingRead;
} GuardedData;

void adc_init();
void adc_read_into(uint8_t *);
void adc_start();
void adc_stop();

void ADC_IRQHandler();
void DMA1_Channel1_IRQHandler();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc_handle);

#endif