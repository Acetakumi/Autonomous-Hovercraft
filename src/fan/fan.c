// fan.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include "fan.h"

// Lift fan  -> PD6 (OC0A)
// Thrust fan -> PD5 (OC0B)

// --------------- micros() variables ---------------
static volatile unsigned long t0_overflows = 0;  // counts Timer0 overflows

// --------------- TIMER0 overflow ISR ---------------
ISR(TIMER0_OVF_vect)
{
    t0_overflows++;     // increment overflow counter
}

// --------------- micros() function ---------------
unsigned long fan_micros(void)
{
    unsigned long ovf;
    uint8_t t;

    uint8_t sreg = SREG;
    cli();

    ovf = t0_overflows;
    t   = TCNT0;

    // overflow flag check
    if ((TIFR0 & (1 << TOV0)) && (t < 255))
        ovf++;

    SREG = sreg;

    // Timer0 (16MHz/64 prescaler):
    // tick = 4 µs
    // overflow = 256 * 4 µs = 1024 µs
    return (ovf * 1024UL) + (t * 4UL);
}

// --------------------------------------------------
// FAN PWM INITIALIZATION
// --------------------------------------------------
void fans_init(void)
{
    // Make PD5 (OC0B) and PD6 (OC0A) outputs
    DDRD |= (1 << PD5) | (1 << PD6);

    // Fast PWM mode (TOP = 255)
    TCCR0A = (1 << WGM01) | (1 << WGM00);

    // Non-inverting PWM for both output channels
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

    // Prescaler = 64 → PWM frequency ~976 Hz
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Start PWM at 0 duty
    OCR0A = 0;
    OCR0B = 0;

    // Enable overflow interrupt for micros()
    TIMSK0 |= (1 << TOIE0);
}

// --------------------------------------------------
// FAN CONTROL FUNCTIONS
// --------------------------------------------------
void fan_lift_set(uint8_t duty)
{
    OCR0A = duty;   // PD6 (OC0A)
}

void fan_thrust_set(uint8_t duty)
{
    OCR0B = duty;   // PD5 (OC0B)
}

void fans_shutdown(void)
{
    OCR0A = 0;
    OCR0B = 0;
}


