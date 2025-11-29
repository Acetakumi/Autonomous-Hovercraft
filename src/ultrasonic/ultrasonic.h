#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

typedef enum {
    US_RIGHT = 0,   // formerly US_FRONT — echo on PD2 / INT0, trig on PB3
    US_LEFT  = 1    // echo on PD3 / INT1, trig on PB5
} us_sensor_t;

// Public globals (accessible from main)
extern uint16_t us_right_cm;
extern uint16_t us_left_cm;

// Initialize pins + interrupts
void us_init(void);

// Trigger one sensor (non-blocking)
void us_trigger(us_sensor_t sensor);

// Get last measured distance (9999 = invalid)
uint16_t us_get_cm(us_sensor_t sensor);

// Blocking helper: measure both sensors
void us_update_all(void);

#endif
