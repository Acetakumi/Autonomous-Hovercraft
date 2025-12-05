#ifndef IR_H
#define IR_H

#include <stdint.h>


void ir_init(void);


void ir_update(void);


uint16_t ir_get_raw(void);


uint16_t ir_get_cm(void);

#endif
