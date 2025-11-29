#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// ---------- Public raw data ----------
extern volatile int16_t imu_ax_raw;
extern volatile int16_t imu_ay_raw;
extern volatile int16_t imu_az_raw;

extern volatile int16_t imu_gx_raw;
extern volatile int16_t imu_gy_raw;
extern volatile int16_t imu_gz_raw;

// ---------- Public converted data ----------
extern volatile float imu_ax_mps2;
extern volatile float imu_ay_mps2;
extern volatile float imu_az_mps2;

extern volatile float imu_accel_total;

// Yaw angle in degrees (integrated gyro Z)
extern volatile float imu_yaw_deg;

// ---------- Public API ----------
void imu_init(void);
void imu_calibrate_gyro(void);
void imu_reset_yaw(void);
void imu_update(void);

#endif // IMU_H
