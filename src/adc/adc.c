#include <avr/io.h>
#include "adc.h"

void adc_init(void)
{
    ADMUX = (1 << REFS0);

    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel)
{
    channel &= 0x07;

    ADMUX = (ADMUX & 0xF0) | channel;

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    return ADC;
}

uint8_t battery_is_low(void)
{
    uint16_t raw = adc_read(VBATT_ADC_CHANNEL);
    return (raw < VBATT_CUTOFF_ADC);
}
