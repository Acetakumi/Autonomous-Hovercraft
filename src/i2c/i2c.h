// i2c.h
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);
uint8_t i2c_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_readAck(void);
uint8_t i2c_readNak(void);

#endif
