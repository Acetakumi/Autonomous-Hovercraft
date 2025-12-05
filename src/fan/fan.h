// fan.h
#ifndef FANS_H
#define FANS_H

#include <stdint.h>


void fans_init(void);

void fan_lift_set(uint8_t duty);
void fan_thrust_set(uint8_t duty);
void fans_shutdown(void);

unsigned long fan_micros(void);

#endif
