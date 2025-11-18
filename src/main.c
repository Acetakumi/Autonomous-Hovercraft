#include <util/delay.h>
#include "fan/fan.h"
#include "adc/adc.h"
#include "uart/uart.h"

int setup(void) {
    UART_begin();
    adc_init();
    fans_init();

    _delay_ms(500);

    return 0;
}

int main(void)
{
    setup();

    while (1)
    {
        if (battery_is_low())
        {
            // Cut both fans when battery below threshold
            fan_lift_set(0);
            fan_thrust_set(0);
        }
        else
        {
            // Example test values
            fan_lift_set(255);     // ~80% lift
            fan_thrust_set(200);   // ~60% thrust
        }

        _delay_ms(100);
    }

    return 0;
}
