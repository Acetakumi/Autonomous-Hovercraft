// ultrasonic.h
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

typedef enum {
    US_FRONT = 0,
    US_LEFT  = 1
} us_sensor_t;

// Init Timer1, pins, and external interrupts (INT0, INT1)
void us_init(void);

// Trigger a 10 µs pulse on the selected sensor (non-blocking except for 10 µs)
void us_trigger(us_sensor_t sensor);

// Get last measured distance in cm for that sensor
uint16_t us_get_cm(us_sensor_t sensor);

// Blocking helper: measure both sensors (front then left),
// with internal delays, and update the globals below.
void us_update_all(void);

// Latest measured distances in cm (updated by us_update_all())
extern uint16_t us_front_cm;
extern uint16_t us_left_cm;

#endif
