#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void servo_init(void);


void servo_set_angle_deg(float angle_deg);


void servo_disable(void);

#endif
