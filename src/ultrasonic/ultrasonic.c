// ultrasonic.c
#include "ultrasonic.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "fan/fan.h"    // for fan_micros()

// ---------- PIN CONFIG ----------
// RIGHT sensor (previously FRONT):
//   TRIG = PB3, ECHO = PD2 (INT0)
#define USR_TRIG_DDR   DDRB
#define USR_TRIG_PORT  PORTB
#define USR_TRIG_PIN   PB3

#define USR_ECHO_DDR   DDRD
#define USR_ECHO_PIN   PD2   // INT0

// LEFT sensor:
//   TRIG = PB5, ECHO = PD3 (INT1)
#define USL_TRIG_DDR   DDRB
#define USL_TRIG_PORT  PORTB
#define USL_TRIG_PIN   PB5

#define USL_ECHO_DDR   DDRD
#define USL_ECHO_PIN   PD3   // INT1

// ---------- INTERNAL STATE ----------

static volatile uint32_t us_right_echo_start_us = 0;
static volatile uint32_t us_left_echo_start_us  = 0;

static volatile uint16_t us_right_last_cm = 9999;
static volatile uint8_t  us_right_echo_state = 0;

static volatile uint16_t us_left_last_cm  = 9999;
static volatile uint8_t  us_left_echo_state = 0;

// Public globals
uint16_t us_right_cm = 9999;
uint16_t us_left_cm  = 9999;

// Convert pulse width (µs) → cm
static uint16_t width_us_to_cm(uint32_t pw)
{
    if (pw == 0 || pw > 30000UL)
        return 9999;

    return (uint16_t)(pw / 58UL);
}

// ---------- INIT ----------

void us_init(void)
{
    // RIGHT
    USR_TRIG_DDR  |= (1 << USR_TRIG_PIN);
    USR_TRIG_PORT &= ~(1 << USR_TRIG_PIN);
    USR_ECHO_DDR  &= ~(1 << USR_ECHO_PIN);

    // LEFT
    USL_TRIG_DDR  |= (1 << USL_TRIG_PIN);
    USL_TRIG_PORT &= ~(1 << USL_TRIG_PIN);
    USL_ECHO_DDR  &= ~(1 << USL_ECHO_PIN);

    // Any logical change on INT0 + INT1
    EICRA |=  (1 << ISC00) | (1 << ISC10);
    EICRA &= ~((1 << ISC01) | (1 << ISC11));

    // Enable interrupts
    EIMSK |= (1 << INT0) | (1 << INT1);

    // Reset state
    us_right_last_cm  = 9999;
    us_left_last_cm   = 9999;
    us_right_cm       = 9999;
    us_left_cm        = 9999;

    us_right_echo_state = 0;
    us_left_echo_state  = 0;
}

// ---------- PUBLIC FUNCTIONS ----------

void us_trigger(us_sensor_t sensor)
{
    if (sensor == US_RIGHT) {

        us_right_echo_state = 0;

        USR_TRIG_PORT |= (1 << USR_TRIG_PIN);
        _delay_us(10);
        USR_TRIG_PORT &= ~(1 << USR_TRIG_PIN);

    } else { // US_LEFT

        us_left_echo_state = 0;

        USL_TRIG_PORT |= (1 << USL_TRIG_PIN);
        _delay_us(10);
        USL_TRIG_PORT &= ~(1 << USL_TRIG_PIN);
    }
}

uint16_t us_get_cm(us_sensor_t sensor)
{
    return (sensor == US_RIGHT) ? us_right_last_cm : us_left_last_cm;
}

void us_update_all(void)
{
    // RIGHT
    us_trigger(US_RIGHT);
    _delay_ms(60);
    us_right_cm = us_get_cm(US_RIGHT);

    // LEFT
    us_trigger(US_LEFT);
    _delay_ms(60);
    us_left_cm  = us_get_cm(US_LEFT);
}

// ---------- INTERRUPTS ----------

// RIGHT sensor echo (INT0, PD2)
ISR(INT0_vect)
{
    uint8_t high = (PIND & (1 << USR_ECHO_PIN)) != 0;

    if (us_right_echo_state == 0) {
        if (high) {
            us_right_echo_start_us = fan_micros();
            us_right_echo_state = 1;
        }
    }
    else {
        if (!high) {
            uint32_t end = fan_micros();
            uint32_t diff;

            if (end >= us_right_echo_start_us)
                diff = end - us_right_echo_start_us;
            else
                diff = (0xFFFFFFFFUL - us_right_echo_start_us) + end + 1;

            us_right_last_cm = width_us_to_cm(diff);
            us_right_echo_state = 0;
        }
    }
}

// LEFT sensor echo (INT1, PD3)
ISR(INT1_vect)
{
    uint8_t high = (PIND & (1 << USL_ECHO_PIN)) != 0;

    if (us_left_echo_state == 0) {
        if (high) {
            us_left_echo_start_us = fan_micros();
            us_left_echo_state = 1;
        }
    }
    else {
        if (!high) {
            uint32_t end = fan_micros();
            uint32_t diff;

            if (end >= us_left_echo_start_us)
                diff = end - us_left_echo_start_us;
            else
                diff = (0xFFFFFFFFUL - us_left_echo_start_us) + end + 1;

            us_left_last_cm = width_us_to_cm(diff);
            us_left_echo_state = 0;
        }
    }
}
