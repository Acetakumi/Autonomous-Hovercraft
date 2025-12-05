#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

typedef enum {
    US_RIGHT = 0,   
    US_LEFT  = 1   
} us_sensor_t;


extern uint16_t us_right_cm;
extern uint16_t us_left_cm;


void us_init(void);


void us_trigger(us_sensor_t sensor);


uint16_t us_get_cm(us_sensor_t sensor);


void us_update_all(void);

#endif
