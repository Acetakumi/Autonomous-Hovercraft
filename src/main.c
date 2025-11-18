#define F_CPU 16000000UL
#include <util/delay.h>
#include "fan.h"
#include "adc.h"

int main(void)
{
    fans_init();
    adc_init();

    _delay_ms(500);

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
