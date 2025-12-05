#ifndef ADC_H
#define ADC_H

#include <stdint.h>


#define VBATT_ADC_CHANNEL    7
#define VBATT_CUTOFF_ADC     10

void adc_init(void);
uint16_t adc_read(uint8_t channel);
uint8_t battery_is_low(void);

#endif
