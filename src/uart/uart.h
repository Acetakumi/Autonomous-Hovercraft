#ifndef UART_H
#define UART_H

#include <stdint.h>

// ===== Public API =====

// Initialize UART at 9600 baud
void UART_begin(void);

// Send a single character
void UART_write(char c);

// Print a C-string over UART
void UART_print(const char *str);

// Print a float with 2 decimals (e.g. 3.14)
// NOTE: Implemented manually (no printf)
void UART_printFloat(float value);

#endif
