// ultrasonic.c
#include "ultrasonic.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>

// ---------- PIN CONFIG ----------
// FRONT sensor on P6:
//   ECHO = PD2 (INT0), TRIG = PB3
#define US1_TRIG_DDR   DDRB
#define US1_TRIG_PORT  PORTB
#define US1_TRIG_PIN   PB3

#define US1_ECHO_DDR   DDRD
#define US1_ECHO_PIN   PD2   // INT0

// LEFT sensor on P13:
//   ECHO = PD3 (INT1), TRIG = PB5
#define US2_TRIG_DDR   DDRB
#define US2_TRIG_PORT  PORTB
#define US2_TRIG_PIN   PB5

#define US2_ECHO_DDR   DDRD
#define US2_ECHO_PIN   PD3   // INT1

// ---------- INTERNAL STATE ----------

// Raw internal last distances (cm), per sensor
static volatile uint16_t us1_last_cm    = 0;
static volatile uint8_t  us1_echo_state = 0;   // 0 = waiting rising, 1 = waiting falling

static volatile uint16_t us2_last_cm    = 0;
static volatile uint8_t  us2_echo_state = 0;

// Public globals (accessed from main)
uint16_t us_front_cm = 0;
uint16_t us_left_cm  = 0;

// ticks -> cm with Timer1 @ F_CPU/8 (16MHz -> 2MHz -> 0.5us/tick)
// distance (cm) ≈ ticks / 116
static uint16_t ticks_to_cm(uint16_t ticks)
{
    return ticks / 116;
}

// ---------- INIT ----------

void us_init(void)
{
    // TRIGs as outputs, low
    US1_TRIG_DDR  |= (1 << US1_TRIG_PIN);
    US1_TRIG_PORT &= ~(1 << US1_TRIG_PIN);

    US2_TRIG_DDR  |= (1 << US2_TRIG_PIN);
    US2_TRIG_PORT &= ~(1 << US2_TRIG_PIN);

    // ECHOs as inputs
    US1_ECHO_DDR &= ~(1 << US1_ECHO_PIN);
    US2_ECHO_DDR &= ~(1 << US2_ECHO_PIN);

    // Timer1 normal mode, prescaler = 8 (0.5 µs per tick)
    TCCR1A = 0x00;
    TCCR1B = (1 << CS11);   // CS11 = 1 → prescaler 8
    TCNT1  = 0;

    // INT0 and INT1 on any logical change (rising + falling)
    // ISC00 = 1, ISC01 = 0 → any edge for INT0
    // ISC10 = 1, ISC11 = 0 → any edge for INT1
    EICRA |=  (1 << ISC00) | (1 << ISC10);
    EICRA &= ~((1 << ISC01) | (1 << ISC11));

    // Enable INT0 and INT1
    EIMSK |= (1 << INT0) | (1 << INT1);

    // Reset states
    us1_echo_state = 0;
    us2_echo_state = 0;
    us1_last_cm    = 0;
    us2_last_cm    = 0;

    us_front_cm    = 0;
    us_left_cm     = 0;
}

// ---------- PUBLIC FUNCTIONS ----------

void us_trigger(us_sensor_t sensor)
{
    if (sensor == US_FRONT) {
        us1_echo_state = 0;   // wait for rising edge

        US1_TRIG_PORT |= (1 << US1_TRIG_PIN);  // HIGH
        _delay_us(10);                          // 10 µs pulse
        US1_TRIG_PORT &= ~(1 << US1_TRIG_PIN); // LOW

    } else { // US_LEFT
        us2_echo_state = 0;

        US2_TRIG_PORT |= (1 << US2_TRIG_PIN);  // HIGH
        _delay_us(10);
        US2_TRIG_PORT &= ~(1 << US2_TRIG_PIN); // LOW
    }
}

uint16_t us_get_cm(us_sensor_t sensor)
{
    if (sensor == US_FRONT) {
        return us1_last_cm;
    } else {
        return us2_last_cm;
    }
}

// Blocking helper: measure both sensors and update us_front_cm / us_left_cm
void us_update_all(void)
{
    // FRONT
    us_trigger(US_FRONT);
    _delay_ms(60);                 // let echo return
    us_front_cm = us_get_cm(US_FRONT);

    // LEFT
    us_trigger(US_LEFT);
    _delay_ms(60);
    us_left_cm = us_get_cm(US_LEFT);
}

// ---------- INTERRUPT HANDLERS ----------

// FRONT sensor echo on INT0 (PD2)
ISR(INT0_vect)
{
    uint8_t echo_high = PIND & (1 << US1_ECHO_PIN);

    if (us1_echo_state == 0) {
        // expecting rising edge (start of echo pulse)
        if (echo_high) {
            TCNT1 = 0;             // start timer
            us1_echo_state = 1;    // now wait for falling
        }
    } else {
        // expecting falling edge (end of echo pulse)
        if (!echo_high) {
            uint16_t ticks = TCNT1;
            us1_last_cm    = ticks_to_cm(ticks);
            us1_echo_state = 0;    // back to idle
        }
    }
}

// LEFT sensor echo on INT1 (PD3)
ISR(INT1_vect)
{
    uint8_t echo_high = PIND & (1 << US2_ECHO_PIN);

    if (us2_echo_state == 0) {
        // expecting rising edge
        if (echo_high) {
            TCNT1 = 0;
            us2_echo_state = 1;
        }
    } else {
        // expecting falling edge
        if (!echo_high) {
            uint16_t ticks = TCNT1;
            us2_last_cm    = ticks_to_cm(ticks);
            us2_echo_state = 0;
        }
    }
}
