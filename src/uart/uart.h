#ifndef UART_H
#define UART_H

#include <stdint.h>


void UART_begin(void);


void UART_write(char c);


void UART_print(const char *str);


void UART_printFloat(float value);

#endif
