#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "uart/uart.h"
#include "ultrasonic/ultrasonic.h"
#include "fan/fan.h"
#include "imu/imu.h"
#include "servo/servo.h"
#include "ir/ir.h"

extern uint16_t us_right_cm;
extern uint16_t us_left_cm;

#define FRONT_STOP_CM 30.0f
#define TURN_ANGLE_DEG 48.0f
#define YAW_TOL_DEG 2.0f

#define SERVO_LEFT_LIMIT -40.0f
#define SERVO_RIGHT_LIMIT 50.0f
#define SERVO_CENTER_ANGLE 10.0f
#define KP_HEADING 2.0f

#define LIFT_PWM 255
#define THRUST_FORWARD_PWM 235
#define THRUST_TURN_PWM 220
#define THRUST_STOP_PWM 0

#define TURN_TIMEOUT_TICKS 200

typedef enum {
    STATE_FORWARD = 0,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} HoverState;

static float heading_target_deg = 0.0f;
static float turn_target_deg = 0.0f;
static uint16_t turn_ticks = 0;

static float normalize(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float clamp(float x, float a, float b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

static uint8_t us_ok(uint16_t cm)
{
    return (cm >= 5 && cm <= 500);
}

static void setup_all(void)
{
    UART_begin();
    us_init();
    fans_init();
    servo_init();
    ir_init();
    imu_init();
    imu_calibrate_gyro();
    imu_update();
    imu_reset_yaw();
    servo_set_angle_deg(SERVO_CENTER_ANGLE);
    fan_lift_set(LIFT_PWM);
    fan_thrust_set(0);
    sei();
}

int main(void)
{
    setup_all();
    HoverState state = STATE_FORWARD;

    while (1)
    {
        imu_update();
        float yaw = normalize(imu_yaw_deg);

        ir_update();
        float front_cm = ir_get_cm();

        switch (state)
        {
            case STATE_FORWARD:
            {
                fan_lift_set(LIFT_PWM);
                fan_thrust_set(THRUST_FORWARD_PWM);

                float err = normalize(heading_target_deg - yaw);
                float s = SERVO_CENTER_ANGLE - KP_HEADING * err;
                s = clamp(s, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                servo_set_angle_deg(s);

                if (front_cm <= FRONT_STOP_CM)
                {
                    fan_thrust_set(0);
                    fan_lift_set(0);
                    servo_set_angle_deg(SERVO_CENTER_ANGLE);
                    _delay_ms(700);

                    us_update_all();
                    imu_reset_yaw();
                    imu_update();
                    yaw = normalize(imu_yaw_deg);

                    uint8_t L = us_ok(us_left_cm);
                    uint8_t R = us_ok(us_right_cm);

                    if (L && !R)
                    {
                        turn_target_deg = TURN_ANGLE_DEG;
                        state = STATE_TURN_LEFT;
                    }
                    else if (!L && R)
                    {
                        turn_target_deg = -TURN_ANGLE_DEG;
                        state = STATE_TURN_RIGHT;
                    }
                    else
                    {
                        if (us_left_cm > us_right_cm)
                        {
                            turn_target_deg = TURN_ANGLE_DEG;
                            state = STATE_TURN_LEFT;
                        }
                        else
                        {
                            turn_target_deg = -TURN_ANGLE_DEG;
                            state = STATE_TURN_RIGHT;
                        }
                    }

                    turn_ticks = 0;
                }

                break;
            }

            case STATE_TURN_LEFT:
            {
                turn_ticks++;
                fan_lift_set(LIFT_PWM);
                fan_thrust_set(THRUST_TURN_PWM);

                float err = normalize(turn_target_deg - yaw);
                float s = SERVO_LEFT_LIMIT;
                if (fabs(err) < 20) s = SERVO_CENTER_ANGLE;
                servo_set_angle_deg(s);

                if (fabs(err) < YAW_TOL_DEG)
                {
                    heading_target_deg = yaw;
                    fan_thrust_set(0);
                    servo_set_angle_deg(SERVO_CENTER_ANGLE);
                    state = STATE_FORWARD;
                }

                if (turn_ticks > TURN_TIMEOUT_TICKS)
                {
                    heading_target_deg = yaw;
                    fan_thrust_set(0);
                    state = STATE_FORWARD;
                }

                break;
            }

            case STATE_TURN_RIGHT:
            {
                turn_ticks++;
                fan_lift_set(LIFT_PWM);
                fan_thrust_set(THRUST_TURN_PWM);

                float err = normalize(turn_target_deg - yaw);
                float s = SERVO_RIGHT_LIMIT;
                if (fabs(err) < 20) s = SERVO_CENTER_ANGLE;
                servo_set_angle_deg(s);

                if (fabs(err) < YAW_TOL_DEG)
                {
                    heading_target_deg = yaw;
                    fan_thrust_set(0);
                    servo_set_angle_deg(SERVO_CENTER_ANGLE);
                    state = STATE_FORWARD;
                }

                if (turn_ticks > TURN_TIMEOUT_TICKS)
                {
                    heading_target_deg = yaw;
                    fan_thrust_set(0);
                    state = STATE_FORWARD;
                }

                break;
            }
        }

        _delay_ms(50);
    }

    return 0;
}
