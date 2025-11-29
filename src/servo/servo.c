#include <avr/io.h>
#include "servo.h"

// -------- CONFIG --------
//
// Servo on OC1A / PB1
// Timer1 Fast PWM with ICR1 as TOP
// F_CPU = 16 MHz, prescaler = 8 -> 0.5 µs per tick
// 20 ms period (50 Hz) -> 20000 µs / 0.5 µs = 40000 ticks
//
// LOGICAL API: angle_deg is around center:
//   angle_deg =   0   -> center
//   angle_deg = -90   -> full left
//   angle_deg = +90   -> full right
//
// We use ~200–2800 µs because you tested it and it gives full travel
// on YOUR specific servo. If it ever buzzes or hits hard stops, back
// these off a bit.

// ---- CALIBRATION CONSTANTS ----
#define SERVO_PULSE_MIN_US      0.0f   // extreme one side
#define SERVO_PULSE_MAX_US      3000.0f  // extreme other side

#define SERVO_ANGLE_MIN_DEG     -90.0f
#define SERVO_ANGLE_MAX_DEG     +90.0f

// microseconds to timer ticks
static inline uint16_t us_to_ticks(float us)
{
    // 0.5 µs per tick at prescaler 8
    return (uint16_t)(us * 2.0f + 0.5f);   // round, don’t just truncate
}

void servo_init(void)
{
    // PB1 (OC1A) as output
    DDRB |= (1 << PB1);

    // Timer1: Fast PWM, mode 14: TOP = ICR1
    // WGM13:0 = 1110 (14)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);               // non-inverting on OC1A, WGM11=1
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11); // prescaler = 8

    // 20 ms period (50 Hz)
    ICR1 = us_to_ticks(20000.0f);   // 40000 ticks

    // Center the servo at init
    servo_set_angle_deg(0.0f);
}

void servo_set_angle_deg(float angle_deg)
{
    // Clamp logical angle to allowed range
    if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;
    if (angle_deg < SERVO_ANGLE_MIN_DEG) angle_deg = SERVO_ANGLE_MIN_DEG;

    // Linear mapping:
    // angle = -90 -> SERVO_PULSE_MIN_US
    // angle = +90 -> SERVO_PULSE_MAX_US
    float span_angle = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;   // 180
    float span_pulse = SERVO_PULSE_MAX_US   - SERVO_PULSE_MIN_US;   // 2600 for 200–2800

    float alpha = (angle_deg - SERVO_ANGLE_MIN_DEG) / span_angle;   // 0..1
    float pulse_us = SERVO_PULSE_MIN_US + alpha * span_pulse;

    // Extra safety clamp
    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;

    OCR1A = us_to_ticks(pulse_us);
}

void servo_disable(void)
{
    // Disconnect OC1A pin from Timer1 (no PWM)
    TCCR1A &= ~(1 << COM1A1);
}
