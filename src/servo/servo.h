#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// Initialize Timer1 for servo PWM on OC1A (PB1)
void servo_init(void);

// Set steering angle in *degrees around center*.
// 0   = straight
// <0  = left
// >0  = right
// Internally clamped to a safe range (about -90..+90).
void servo_set_angle_deg(float angle_deg);

// Optional: fully stop PWM output (servo off / float)
void servo_disable(void);

#endif
