// fan.c
#include <avr/io.h>
#include <avr/interrupt.h>
#include "fan.h"


static volatile unsigned long t0_overflows = 0; 


ISR(TIMER0_OVF_vect)
{
    t0_overflows++;     
}


unsigned long fan_micros(void)
{
    unsigned long ovf;
    uint8_t t;

    uint8_t sreg = SREG;
    cli();

    ovf = t0_overflows;
    t   = TCNT0;

  
    if ((TIFR0 & (1 << TOV0)) && (t < 255))
        ovf++;

    SREG = sreg;

   
    return (ovf * 1024UL) + (t * 4UL);
}


void fans_init(void)
{
   
    DDRD |= (1 << PD5) | (1 << PD6);


    TCCR0A = (1 << WGM01) | (1 << WGM00);

   
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

   
    TCCR0B = (1 << CS01) | (1 << CS00);

  
    OCR0A = 0;
    OCR0B = 0;

    
    TIMSK0 |= (1 << TOIE0);
}


void fan_lift_set(uint8_t duty)
{
    OCR0A = duty;   
}

void fan_thrust_set(uint8_t duty)
{
    OCR0B = duty; 
}

void fans_shutdown(void)
{
    OCR0A = 0;
    OCR0B = 0;
}


