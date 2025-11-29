#ifndef IR_H
#define IR_H

#include <stdint.h>

// Initialize ADC + IR sensor channel
void ir_init(void);

// Read sensor (ADC + distance). Call once per main loop.
void ir_update(void);

// Raw 10-bit ADC value (0–1023)
uint16_t ir_get_raw(void);

// Distance estimate in cm (5–80 cm typical). If invalid → returns 9999.
uint16_t ir_get_cm(void);

#endif
