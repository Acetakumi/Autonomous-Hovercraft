#include <avr/io.h>
#include "i2c.h"

// Standard I2C clock
#define SCL_FREQ 100000UL  // 100 kHz

void i2c_init(void)
{
    // SCL = F_CPU / (16 + 2*TWBR*prescaler)
    TWSR = 0x00;         // prescaler = 1
    TWBR = ((F_CPU / SCL_FREQ) - 16) / 2;
}

uint8_t i2c_start(uint8_t address)
{
    // Send START
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    // Send address
    TWDR = address;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    return 0;
}

void i2c_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

uint8_t i2c_readAck(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t i2c_readNak(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}
