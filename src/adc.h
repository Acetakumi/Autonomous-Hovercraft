#ifndef ADC_H
#define ADC_H

#include <stdint.h>

// ADC channel used for battery voltage sensing (via divider)
#define VBATT_ADC_CHANNEL    7

// Raw ADC threshold for cutoff (from your slide: if ADC7 < 10 → < 12 V)
#define VBATT_CUTOFF_ADC     10

// Initialize ADC hardware.
// - Uses external AREF (DO NOT change board's Aref config)
// - Prescaler = 128 → 16 MHz / 128 = 125 kHz ADC clock
void adc_init(void);

// Read a 10-bit ADC value (0–1023) from channel 0–7.
uint16_t adc_read(uint8_t channel);

// Convenience helper for battery cutoff logic.
// Returns 1 if battery is considered LOW (below cutoff), else 0.
uint8_t battery_is_low(void);

#endif
