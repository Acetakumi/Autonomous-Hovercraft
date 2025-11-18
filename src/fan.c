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

    // -------- Timer0 setup --------
    // Fast PWM (8-bit) on both channels
    // COM0A1:0 = 10 (non-inverting on OC0A)
    // COM0B1:0 = 10 (non-inverting on OC0B)
    // WGM01:0 = 11 (Fast PWM)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) |
             (1 << WGM01)  | (1 << WGM00);

    // WGM02 = 0, Prescaler = 64
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Start fans off
    OCR0A = 0;   // lift fan
    OCR0B = 0;   // thrust fan
}

void fan_lift_set(uint8_t duty)
{
    OCR0A = duty;   // PD6, OC0A
}

void fan_thrust_set(uint8_t duty)
{
    OCR0B = duty;   // PD5, OC0B
}
