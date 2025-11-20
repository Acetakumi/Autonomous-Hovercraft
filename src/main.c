// #include <util/delay.h>
// #include "fan/fan.h"
// #include "adc/adc.h"
// #include "uart/uart.h"

// int setup(void) {
//     UART_begin();
//     adc_init();
//     fans_init();

//     _delay_ms(500);

//     return 0;
// }

// int main(void)
// {
//     setup();

//     while (1)
//     {
//         if (battery_is_low())
//         {
//             // Cut both fans when battery below threshold
//             fan_lift_set(0);
//             fan_thrust_set(0);
//         }
//         else
//         {
//             // Example test values
//             fan_lift_set(255);     // ~80% lift
//             fan_thrust_set(255);   // ~60% thrust
//         }

//         _delay_ms(100);
//     }

//     return 0;
// }

// main.c

// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <util/delay.h>

// #include "uart/uart.h"
// #include "ultrasonic.h"

// int main(void)
// {
//     UART_begin();
//     UART_print("Ultrasonic 2-sensor test starting...\r\n");

//     us_init();   // init sensors + timer1 + interrupts
//     sei();        // enable global interrupts

//     while (1)
//     {
//         // --- Update both sensors (blocking ~120 ms) ---
//         us_update_all();

//         // --- Print FRONT distance ---
//         UART_print("Front: ");
//         UART_printFloat((float)us_front_cm);
//         UART_print(" cm\r\n");

//         // --- Print LEFT distance ---
//         UART_print("Left:  ");
//         UART_printFloat((float)us_left_cm);
//         UART_print(" cm\r\n");

//         UART_print("----------------------\r\n");

//         _delay_ms(200); // avoid spamming serial monitor
//     }
// }




#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart/uart.h"
#include "ultrasonic/ultrasonic.h"
#include "fan/fan.h"
#include "adc/adc.h"
#include "ir/ir.h"
#include "imu/imu.h"

void setup(void)
{
    UART_begin();
    UART_print("System starting...\r\n");

    us_init();
    fans_init();
    adc_init();
    ir_init();
    imu_init();
    UART_print("Calibrating gyro... keep hovercraft STILL\r\n");
    imu_calibrate_gyro();
    UART_print("Done calibration\r\n");
          // init ADC if you use battery_is_low()

    sei();               // enable global interrupts
}

int on_fans()
{
    if (battery_is_low() || ir_bar_detected()) {
        fans_shutdown(); 
        return 0; // battery low → kill fans
    } else {
        fans_update();
        return 1;     // battery OK → run normal logic (both fans on for now)
    }
}

int main(void)
{
    setup();

    while (1)
    {
        // Update sensors first
        us_update_all(); 

        ir_update(); 

        imu_update();  
       
        
        // updates us_front_cm, us_left_cm

        // Then decide what to do with fans
        // if (!(on_fans())) {
        //     UART_print("Fans OFF due to low battery or IR bar detected!\r\n");
        //     break;
        // }

        // Debug print
        UART_print("Front: ");
        UART_printFloat((float)us_front_cm);
        UART_print(" cm | ");

        UART_print("Left: ");
        UART_printFloat((float)us_left_cm);

         UART_print(" | IR raw: ");
        UART_printFloat((float)ir_front_raw);

        UART_print(" | Yaw: ");
        UART_printFloat(imu_yaw_deg);

        UART_print(" | a_total: ");
        UART_printFloat(imu_accel_total - 9.81f); // remove gravity  

            UART_print(" cm\r\n");

        _delay_ms(200);
    }

    return 0;
}







