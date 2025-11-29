// fan.h
#ifndef FANS_H
#define FANS_H

#include <stdint.h>

// Initialize PWM + micros()
void fans_init(void);

// Fan control
void fan_lift_set(uint8_t duty);
void fan_thrust_set(uint8_t duty);
void fans_shutdown(void);

// micros() provided by this module
unsigned long fan_micros(void);

#endif
