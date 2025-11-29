#include <avr/io.h>
#include "adc.h"

/**
 * Initialize ADC:
 * AREF = external reference (board provides it via Aref pin)
 * Right-adjusted result
 * ADC enabled, prescaler = 128 (125 kHz ADC clock)
 */
void adc_init(void)
{
    // REFS1:0 = 00 → external AREF
    // ADLAR = 0 → right-adjusted result
    ADMUX = 0x00;

    // ADEN = 1 (enable ADC)
    // ADPS2:0 = 111 → prescaler 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// ----------------------------------------------------------
// Read ADC channel: 0–7
// Blocks until conversion completes.
// ----------------------------------------------------------
uint16_t adc_read(uint8_t channel)
{
    // Limit to lower 3 bits (0–7)
    channel &= 0x07;

    // Keep reference bits, change only MUX[2:0]
    ADMUX = (ADMUX & 0xF0) | channel;

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));

    // Return 10-bit result
    return ADC;
}

// ----------------------------------------------------------
// Battery cutoff helper:
// Uses VBATT_ADC_CHANNEL (7) and VBATT_CUTOFF_ADC (10).
// Returns 1 if battery is "low" (below threshold), else 0.
//
// This matches the lecture slide logic:
// if (readADC(7) < 10) shutdown();
// ----------------------------------------------------------
uint8_t battery_is_low(void)
{
    uint16_t raw = adc_read(VBATT_ADC_CHANNEL);
    return (raw < VBATT_CUTOFF_ADC) ? 1 : 0;
}