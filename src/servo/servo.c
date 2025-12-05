#include <avr/io.h>
#include "servo.h"

#define SERVO_PULSE_MIN_US      0.0f
#define SERVO_PULSE_MAX_US      3000.0f

#define SERVO_ANGLE_MIN_DEG     -90.0f
#define SERVO_ANGLE_MAX_DEG     +90.0f

static inline uint16_t us_to_ticks(float us)
{
    return (uint16_t)(us * 2.0f + 0.5f);
}

void servo_init(void)
{
    DDRB |= (1 << PB1);

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11);

    ICR1 = us_to_ticks(20000.0f);

    servo_set_angle_deg(0.0f);
}

void servo_set_angle_deg(float angle_deg)
{
    if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;
    if (angle_deg < SERVO_ANGLE_MIN_DEG) angle_deg = SERVO_ANGLE_MIN_DEG;

    float span_angle = SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG;
    float span_pulse = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;

    float alpha = (angle_deg - SERVO_ANGLE_MIN_DEG) / span_angle;
    float pulse_us = SERVO_PULSE_MIN_US + alpha * span_pulse;

    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > SERVO_PULSE_MAX_US) pulse_us = SERVO_PULSE_MAX_US;

    OCR1A = us_to_ticks(pulse_us);
}

void servo_disable(void)
{
    TCCR1A &= ~(1 << COM1A1);
}
