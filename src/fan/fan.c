#include <avr/io.h>
#include "fan.h"

// ----------------------------------------------------------
// Timer0 dual-channel PWM:
// - OC0A (PD6) = lift fan
// - OC0B (PD5) = thrust fan
//
// Fast PWM 8-bit
// Fpwm ≈ 16MHz / (64 * 256) ≈ 976 Hz
// ----------------------------------------------------------

void fans_init(void)
{
    // Set PD6 (OC0A) and PD5 (OC0B) as outputs
    DDRD |= (1 << DDD6) | (1 << DDD5);

    // Fast PWM on both channels, non-inverting
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) |
             (1 << WGM01)  | (1 << WGM00);

    // Prescaler = 64
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Start fans OFF
    OCR0A = 0;
    OCR0B = 0;
}

void fan_lift_set(uint8_t duty)
{
    OCR0A = duty;   // OC0A → PD6
}

void fan_thrust_set(uint8_t duty)
{
    OCR0B = duty;   // OC0B → PD5
}

// NEW FUNCTION: cut both fans immediately
void fans_shutdown(void)
{
    OCR0A = 0;
    OCR0B = 0;
}

// Main fan logic (for now: turn both ON)
void fans_update(void)
{
    // For now both fans ON at full power
    fan_lift_set(255);
    fan_thrust_set(255);
}

