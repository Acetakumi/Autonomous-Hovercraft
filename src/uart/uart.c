#include <avr/io.h>
#include "uart.h"

#define UART_BAUD      9600UL
#define UART_UBRR_VAL  ((F_CPU / (16UL * UART_BAUD)) - 1UL)

void UART_begin(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(UART_UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UART_UBRR_VAL & 0xFF);

    // Normal speed (U2X0 = 0)
    UCSR0A &= ~(1 << U2X0);

    // Frame: 8-N-1
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Enable TX and RX
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
}

void UART_write(char c)
{
    // Wait until transmit buffer is empty
    while (!(UCSR0A & (1 << UDRE0)))
        ;

    // Send the byte
    UDR0 = (uint8_t)c;
}

void UART_print(const char *str)
{
    while (*str)
    {
        UART_write(*str);
        str++;
    }
}

void UART_printFloat(float value)
{
  
    if (value < 0)
    {
        UART_write('-');
        value = -value;
    }

    
    int intPart = (int)value;
    float frac = value - intPart;

   
    char buffer[10];
    int i = 0;

    if (intPart == 0)
    {
        UART_write('0');
    }
    else
    {
        while (intPart > 0)
        {
            buffer[i++] = '0' + (intPart % 10);
            intPart /= 10;
        }
        while (i--)
            UART_write(buffer[i]);
    }

    UART_write('.');

    int fracInt = (int)(frac * 100 + 0.5f);

    UART_write('0' + (fracInt / 10));
    UART_write('0' + (fracInt % 10));
}
