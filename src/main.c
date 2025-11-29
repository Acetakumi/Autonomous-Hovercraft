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

// ---------- EXTERNS FROM ultrasonic.c ----------
extern uint16_t us_right_cm;
extern uint16_t us_left_cm;

// ---------- CONSTANTS ----------

// IR-based obstacle distances (cm, using ir_get_cm())
#define FRONT_STOP_CM       30.0f   // at this distance: stop lift + thrust, then turn

// Turning
#define TURN_ANGLE_DEG      25.0f   // target yaw for turn
#define YAW_TOL_DEG         2.0f    // how close to target to stop turn

// Turn staging thresholds (fractions of TURN_ANGLE_DEG)
#define TURN_ERR_FAR_DEG    (0.5f * TURN_ANGLE_DEG)   // > 10° = hard steering
#define TURN_ERR_MED_DEG    (0.2f * TURN_ANGLE_DEG)   // > 4° = medium steering

// Servo logical angles (-90..+90 range)
#define SERVO_LEFT_LIMIT        -70.0f   // safe left limit
#define SERVO_RIGHT_LIMIT       +60.0f   // safe right limit
#define SERVO_CENTER_ANGLE      10.0f    // <- THIS is your physical straight-ahead
#define SERVO_RIGHT_MED_ANGLE   20.0f 
#define SERVO_LEFT_MED_ANGLE   -20.0f   // medium angles to soften turns

// Heading-hold (for straight line)
#define SERVO_TRIM_DEG      0.0f       // no extra trim; center is already 10°
#define KP_HEADING          2.0f       // correction for drift

// Turn steering gain (kept for possible future use)
#define K_TURN_SERVO        2.5f

// Fans
#define LIFT_PWM                255
#define THRUST_FORWARD_PWM      255

// Turn speeds
#define THRUST_TURN_PWM_FAST    200
#define THRUST_TURN_PWM_SLOW    130
#define THRUST_STOP_PWM         0

// Cooldown after turn
#define POST_TURN_TICKS     12

// Safety / timing
#define TURN_TIMEOUT_TICKS  30     
#define TURN_MIN_TICKS      10      

// US sanity
#define US_MIN_VALID_CM     5
#define US_MAX_VALID_CM     300

// ---------- STATES ----------
typedef enum {
    STATE_FORWARD = 0,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT
} HoverState;

// ---------- GLOBALS ----------
static float   heading_target_deg = 0.0f;
static float   turn_target_deg    = 0.0f;
static uint8_t post_turn_ticks    = 0;
static uint16_t turn_ticks        = 0;

// ---------- IR SMOOTHING ----------
#define IR_MEAN_WINDOW 5
static float   ir_buf[IR_MEAN_WINDOW];
static uint8_t ir_buf_idx   = 0;
static uint8_t ir_buf_count = 0;

// ---------- IR STARTUP IGNORE ----------
#define IR_IGNORE_LOOPS    50      // ignore IR for first N main loops
static uint16_t loop_count = 0;    // increments every loop

// ---------- HELPERS ----------

static float normalize_angle_deg(float a)
{
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static uint8_t us_valid(uint16_t cm)
{
    return (cm >= US_MIN_VALID_CM && cm <= US_MAX_VALID_CM);
}

static float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// Update IR, push into buffer, return mean cm
static float ir_update_and_get_mean_cm(void)
{
    ir_update();
    float cm = (float) ir_get_cm();

    ir_buf[ir_buf_idx] = cm;
    ir_buf_idx = (ir_buf_idx + 1) % IR_MEAN_WINDOW;
    if (ir_buf_count < IR_MEAN_WINDOW)
        ir_buf_count++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < ir_buf_count; i++)
        sum += ir_buf[i];

    return sum / (float)ir_buf_count;
}

static void setup(void)
{
    UART_begin();
    UART_print("Hovercraft IR+IMU+US navigation (20deg turns, IR mean, staged angles, center=10deg)\r\n");

    us_init();
    fans_init();
    servo_init();
    ir_init();
    imu_init();

    UART_print("Calibrating gyro...\r\n");
    imu_calibrate_gyro();
    UART_print("Gyro OK\r\n");

    imu_update();
    imu_reset_yaw();

    heading_target_deg = 0.0f;

    // Center servo at *true* straight-ahead
    servo_set_angle_deg(SERVO_CENTER_ANGLE);

    // Lift ON, thrust OFF at boot
    fan_lift_set(LIFT_PWM);
    fan_thrust_set(THRUST_STOP_PWM);

    sei();
}

// ---------- MAIN ----------
int main(void)
{
    setup();

    HoverState state = STATE_FORWARD;

    while (1)
    {
        // increment loop counter EVERY loop
        loop_count++;

        imu_update();

        float yaw = normalize_angle_deg(imu_yaw_deg);

        if (yaw > 720.0f || yaw < -720.0f)
        {
            UART_print("WARN: yaw out of crazy range, resetting\r\n");
            imu_reset_yaw();
            imu_update();
            yaw = normalize_angle_deg(imu_yaw_deg);
        }

        // Get smoothed IR distance (we can read it, but we won't ACT on it until loop_count > IR_IGNORE_LOOPS)
        float front_cm = ir_update_and_get_mean_cm();

        switch (state)
        {
            // ============================
            //   FORWARD
            // ============================
            case STATE_FORWARD:
            {
                // Lift always ON in straight
                fan_lift_set(LIFT_PWM);

                // Thrust & heading control
                if (post_turn_ticks > 0)
                {
                    // Cooldown after turn: softer thrust, pure center
                    post_turn_ticks--;
                    fan_thrust_set(THRUST_TURN_PWM_FAST);

                    float servo_cmd = SERVO_CENTER_ANGLE;  // pure center during cooldown
                    servo_cmd = clampf(servo_cmd, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                    servo_set_angle_deg(servo_cmd);
                }
                else
                {
                    fan_thrust_set(THRUST_FORWARD_PWM);

                    float err = normalize_angle_deg(heading_target_deg - yaw);
                    if (fabsf(err) < 1.0f) err = 0.0f;

                    float servo_cmd = clampf(
                        SERVO_CENTER_ANGLE + SERVO_TRIM_DEG - KP_HEADING * err, 
                        SERVO_LEFT_LIMIT, 
                        SERVO_RIGHT_LIMIT
                    );
                    
                    servo_set_angle_deg(servo_cmd);
                }

                // ---------- HARD STOP + TURN DECISION (using MEAN IR) ----------
                // We ONLY allow IR to trigger after IR_IGNORE_LOOPS main loops.
                if (loop_count > IR_IGNORE_LOOPS)
                {
                    if (post_turn_ticks == 0 && front_cm <= FRONT_STOP_CM)
                    {
                        UART_print("OBSTACLE (MEAN IR): front_ir_cm = ");
                        UART_printFloat(front_cm);
                        UART_print(" -> HARD STOP (lift+thrust)\r\n");

                        // FULL STOP: kill thrust + lift, keep servo straight
                        fan_thrust_set(THRUST_STOP_PWM);
                        fan_lift_set(0);
                        servo_set_angle_deg(SERVO_CENTER_ANGLE);
                        _delay_ms(1000);   // let it settle flat

                        // Get left/right space with ultrasonics
                        us_update_all();

                        UART_print("US Right: ");
                        UART_printFloat((float)us_right_cm);
                        UART_print(" cm | US Left: ");
                        UART_printFloat((float)us_left_cm);
                        UART_print(" cm\r\n");

                        uint8_t left_ok  = us_valid(us_left_cm);
                        uint8_t right_ok = us_valid(us_right_cm);

                        // Reset yaw so 0° = current forward
                        imu_reset_yaw();
                        imu_update();
                        yaw = normalize_angle_deg(imu_yaw_deg);

                        turn_ticks = 0;

                        // Decide direction
                        if (left_ok && !right_ok)
                        {
                            turn_target_deg = +TURN_ANGLE_DEG;
                            state = STATE_TURN_LEFT;
                            UART_print("Decision: TURN LEFT (right invalid)\r\n");
                        }
                        else if (!left_ok && right_ok)
                        {
                            turn_target_deg = -TURN_ANGLE_DEG;
                            state = STATE_TURN_RIGHT;
                            UART_print("Decision: TURN RIGHT (left invalid)\r\n");
                        }
                        else
                        {
                            if (us_left_cm > us_right_cm)
                            {
                                turn_target_deg = +TURN_ANGLE_DEG;
                                state = STATE_TURN_LEFT;
                                UART_print("Decision: TURN LEFT\r\n");
                            }
                            else
                            {
                                turn_target_deg = -TURN_ANGLE_DEG;
                                state = STATE_TURN_RIGHT;
                                UART_print("Decision: TURN RIGHT\r\n");
                            }
                        }

                        // Before entering turn, restore lift (thrust is handled in TURN state)
                        fan_lift_set(LIFT_PWM);
                        fan_thrust_set(THRUST_STOP_PWM);
                    }
                }

                break;
            }

            // ============================
            //   TURN LEFT (aim +TURN_ANGLE_DEG)
            // ============================
            case STATE_TURN_LEFT:
            {
                // Lift ON during turn
                fan_lift_set(LIFT_PWM);

                turn_ticks++;

                float err = normalize_angle_deg(turn_target_deg - yaw);
                float abs_err = fabsf(err);

                // Speed: faster when far, slower when close
                uint8_t thrust_turn =
                    (abs_err > TURN_ERR_FAR_DEG) ? THRUST_TURN_PWM_FAST : THRUST_TURN_PWM_SLOW;

                fan_thrust_set(thrust_turn);

                // ---- STAGED SERVO USING MED ANGLES ----
                float servo_f;

                if (err > 0.0f)
                {
                    // We still need to turn LEFT
                    if (abs_err > TURN_ERR_FAR_DEG)
                    {
                        // Far from target: hard left
                        servo_f = SERVO_LEFT_LIMIT;
                    }
                    else if (abs_err > TURN_ERR_MED_DEG)
                    {
                        // Closer: medium left
                        servo_f = SERVO_LEFT_MED_ANGLE;
                    }
                    else if (abs_err > YAW_TOL_DEG)
                    {
                        // Very close but not done: gentle left (half-medium)
                        servo_f = SERVO_LEFT_MED_ANGLE * 0.5f;   // ≈ -7.5°
                    }
                    else
                    {
                        servo_f = SERVO_CENTER_ANGLE;
                    }
                }
                else
                {
                    // Overshoot (err <= 0), small correction to the right
                    if (abs_err > TURN_ERR_MED_DEG)
                    {
                        servo_f = SERVO_RIGHT_MED_ANGLE;
                    }
                    else if (abs_err > YAW_TOL_DEG)
                    {
                        servo_f = SERVO_RIGHT_MED_ANGLE * 0.5f;  // ≈ +7.5°
                    }
                    else
                    {
                        servo_f = SERVO_CENTER_ANGLE;
                    }
                }

                servo_f = clampf(servo_f, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                servo_set_angle_deg(servo_f);

                if (turn_ticks > TURN_MIN_TICKS && abs_err < YAW_TOL_DEG)
                {
                    UART_print("LEFT TURN DONE, yaw=");
                    UART_printFloat(yaw);
                    UART_print("\r\n");

                    fan_thrust_set(THRUST_STOP_PWM);
                    _delay_ms(200);

                    imu_reset_yaw();
                    imu_update();
                    heading_target_deg = 0.0f;

                    // After turn: pure center (10°)
                    servo_set_angle_deg(SERVO_CENTER_ANGLE);

                    post_turn_ticks = POST_TURN_TICKS;
                    state = STATE_FORWARD;
                }
                else if (turn_ticks > TURN_TIMEOUT_TICKS)
                {
                    UART_print("WARN: LEFT turn timeout, forcing FORWARD\r\n");
                    fan_thrust_set(THRUST_STOP_PWM);
                    _delay_ms(200);

                    imu_reset_yaw();
                    imu_update();
                    heading_target_deg = 0.0f;

                    servo_set_angle_deg(SERVO_CENTER_ANGLE);

                    post_turn_ticks = POST_TURN_TICKS;
                    state = STATE_FORWARD;
                }
                break;
            }

            // ============================
            //   TURN RIGHT (aim -TURN_ANGLE_DEG)
            // ============================
            case STATE_TURN_RIGHT:
            {
                fan_lift_set(LIFT_PWM);

                turn_ticks++;

                float err = normalize_angle_deg(turn_target_deg - yaw);
                float abs_err = fabsf(err);

                uint8_t thrust_turn =
                    (abs_err > TURN_ERR_FAR_DEG) ? THRUST_TURN_PWM_FAST : THRUST_TURN_PWM_SLOW;

                fan_thrust_set(thrust_turn);

                // ---- STAGED SERVO USING MED ANGLES ----
                float servo_f;

                if (err < 0.0f)
                {
                    // Still need to turn RIGHT (target is negative)
                    if (abs_err > TURN_ERR_FAR_DEG)
                    {
                        // Far from target: hard right
                        servo_f = SERVO_RIGHT_LIMIT;
                    }
                    else if (abs_err > TURN_ERR_MED_DEG)
                    {
                        // Closer: medium right
                        servo_f = SERVO_RIGHT_MED_ANGLE;
                    }
                    else if (abs_err > YAW_TOL_DEG)
                    {
                        // Very close, gentle right
                        servo_f = SERVO_RIGHT_MED_ANGLE * 0.5f; // ≈ +7.5°
                    }
                    else
                    {
                        servo_f = SERVO_CENTER_ANGLE;
                    }
                }
                else
                {
                    // Overshoot (err >= 0), small correction to the left
                    if (abs_err > TURN_ERR_MED_DEG)
                    {
                        servo_f = SERVO_LEFT_MED_ANGLE;
                    }
                    else if (abs_err > YAW_TOL_DEG)
                    {
                        servo_f = SERVO_LEFT_MED_ANGLE * 0.5f; // ≈ -7.5°
                    }
                    else
                    {
                        servo_f = SERVO_CENTER_ANGLE;
                    }
                }

                servo_f = clampf(servo_f, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                servo_set_angle_deg(servo_f);

                if (turn_ticks > TURN_MIN_TICKS && abs_err < YAW_TOL_DEG)
                {
                    UART_print("RIGHT TURN DONE, yaw=");
                    UART_printFloat(yaw);
                    UART_print("\r\n");

                    fan_thrust_set(THRUST_STOP_PWM);
                    _delay_ms(200);

                    imu_reset_yaw();
                    imu_update();
                    heading_target_deg = 0.0f;

                    servo_set_angle_deg(SERVO_CENTER_ANGLE);

                    post_turn_ticks = POST_TURN_TICKS;
                    state = STATE_FORWARD;
                }
                else if (turn_ticks > TURN_TIMEOUT_TICKS)
                {
                    UART_print("WARN: RIGHT turn timeout, forcing FORWARD\r\n");
                    fan_thrust_set(THRUST_STOP_PWM);
                    _delay_ms(200);

                    imu_reset_yaw();
                    imu_update();
                    heading_target_deg = 0.0f;

                    servo_set_angle_deg(SERVO_CENTER_ANGLE);

                    post_turn_ticks = POST_TURN_TICKS;
                    state = STATE_FORWARD;
                }
                break;
            }
        }

        // ---- GLOBAL DEBUG ----
        UART_print("STATE=");
        switch (state) {
            case STATE_FORWARD:    UART_print("FWD");  break;
            case STATE_TURN_LEFT:  UART_print("LEFT"); break;
            case STATE_TURN_RIGHT: UART_print("RIGHT");break;
        }
        UART_print(" | yaw_raw=");
        UART_printFloat(imu_yaw_deg);
        UART_print(" | yaw_norm=");
        UART_printFloat(yaw);
        UART_print(" | front_ir_mean_cm=");
        UART_printFloat(front_cm);
        UART_print(" | PT=");
        UART_printFloat((float)post_turn_ticks);
        UART_print(" | TT=");
        UART_printFloat((float)turn_ticks);
        UART_print(" | LC=");
        UART_printFloat((float)loop_count);
        UART_print("\r\n");

        _delay_ms(50);   // IMU_DT must be 0.05f in imu.c
    }

    return 0;
}
