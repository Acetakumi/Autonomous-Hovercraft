// imu.h
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// --------- Public API ---------

// Call once at startup

void imu_calibrate_gyro(void);

void imu_init(void);

// Call at a FIXED rate (dt is defined in imu.c as IMU_DT)
void imu_update(void);

// Reset yaw angle to 0 degrees
void imu_reset_yaw(void);

// --------- Raw sensor data (LSB units) ---------
extern volatile int16_t imu_ax_raw;
extern volatile int16_t imu_ay_raw;
extern volatile int16_t imu_az_raw;

extern volatile int16_t imu_gx_raw;
extern volatile int16_t imu_gy_raw;
extern volatile int16_t imu_gz_raw;

// --------- Converted acceleration (m/s^2) ---------
extern volatile float imu_ax_mps2;
extern volatile float imu_ay_mps2;
extern volatile float imu_az_mps2;

// Magnitude of acceleration vector (sqrt(ax^2 + ay^2 + az^2)), in m/s^2
extern volatile float imu_accel_total;

// --------- Orientation ---------
// Yaw angle in degrees, integrated from gyro Z
extern volatile float imu_yaw_deg;

#endif
