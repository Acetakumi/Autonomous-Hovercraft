// ir.c
#include <avr/io.h>
#include "ir.h"


// Front IR sensor on ADC2 (PC2, P14 pin 1)
#define IR_FRONT_ADC_CHANNEL  2

// Threshold for bar detection (TUNE THIS)
#define IR_BAR_THRESHOLD  400

uint16_t ir_front_raw = 0;

// ---------- INTERNAL: select ADC channel ----------
static void ir_select_channel(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
}

// ---------- PUBLIC FUNCTIONS ----------
void ir_init(void)
{
    // AVcc reference (5V)
    ADMUX = (1 << REFS0);      // REFS0=1, REFS1=0 → AVcc

    // Enable ADC, prescaler = 128 (16MHz/128 = 125 kHz)
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ir_select_channel(IR_FRONT_ADC_CHANNEL);
}

void ir_update(void)
{
    ir_select_channel(IR_FRONT_ADC_CHANNEL);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to finish
    while (ADCSRA & (1 << ADSC))
        ;

    ir_front_raw = ADC;   // 10-bit result
}

// Return 1 if the horizontal bar is detected
uint8_t ir_bar_detected(void)
{
    // For Sharp IR: closer object → HIGHER ADC value
    // If IR sees bar close → ir_front_raw goes above threshold.
    return (ir_front_raw > IR_BAR_THRESHOLD) ? 1 : 0;
}
