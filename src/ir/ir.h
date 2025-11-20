// ir.h
#ifndef IR_H
#define IR_H

#include <stdint.h>

void ir_init(void);
void ir_update(void);

// Last raw ADC reading (0–1023)
extern uint16_t ir_front_raw;

// Returns 1 if the horizontal bar is detected, 0 otherwise
uint8_t ir_bar_detected(void);

#endif
