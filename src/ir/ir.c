
#include <avr/io.h>
#include "ir.h"

// ---------- CONFIG ----------
#define IR_ADC_CHANNEL   2   // PC2 / ADC2
#define IR_MIN_CM        5
#define IR_MAX_CM        80

// Internal storage
static uint16_t ir_raw_value = 0;
static uint16_t ir_distance_cm = 9999;

// ---------- INTERNAL: Select ADC channel ----------
static inline void ir_select_channel(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
}

// ---------- PUBLIC ----------
void ir_init(void)
{
    // AVcc reference, input = ADC2 initially
    ADMUX = (1 << REFS0);

    // Enable ADC, prescaler 128 → 16MHz / 128 = 125kHz ADC clock
    ADCSRA =
        (1 << ADEN) |
        (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    ir_select_channel(IR_ADC_CHANNEL);
}

void ir_update(void)
{
    ir_select_channel(IR_ADC_CHANNEL);

    // Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait until conversion finishes
    while (ADCSRA & (1 << ADSC));

    ir_raw_value = ADC;     // Raw ADC value (0–1023)

    // ----- Distance Formula -----
    // Your original working model was:
    //
    //   dist = (6787 / (raw - 3)) - 4
    //
    // This formula works for typical Sharp GP2Y analog IR sensors.
    //----------------------------------------------

    if (ir_raw_value <= 3) {
        ir_distance_cm = 9999;
        return;
    }

    float d = (6787.0f / (ir_raw_value - 3.0f)) - 4.0f;

    if (d < IR_MIN_CM || d > 200.0f) {
        ir_distance_cm = 9999; // invalid
    }
    else {
        ir_distance_cm = (uint16_t)d;
    }
}

uint16_t ir_get_raw(void)
{
    return ir_raw_value;
}

uint16_t ir_get_cm(void)
{
    return ir_distance_cm;
}
