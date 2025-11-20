#ifndef FANS_H
#define FANS_H

#include <stdint.h>

// Initialize PWM for BOTH fans (lift: PD6, thrust: PD5)
void fans_init(void);

// Set lift fan speed (0–255)
void fan_lift_set(uint8_t duty);

// Set thrust fan speed (0–255)
void fan_thrust_set(uint8_t duty);

void fans_update(void);

void fans_shutdown(void);

#endif
