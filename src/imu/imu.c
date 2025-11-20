// imu.c
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "imu.h"
#include "i2c/i2c.h"


// ---------- MPU-6050 DEFINES ----------
#define MPU_ADDR            0x68    // AD0 low -> 0x68

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B    // start of accel/gyro data block

// Sensitivities for default ranges:
// ±2g  -> 16384 LSB / g
// ±250 dps -> 131 LSB / (deg/s)
#define ACCEL_SENS          16384.0f
#define GYRO_SENS           131.0f

// You MUST call imu_update() at this period (seconds)
// If your main loop uses _delay_ms(200), set this to 0.20f.
// If you call it every 10 ms, set this to 0.01f.
#define IMU_DT              0.20f   // <- change if you change your update rate

// ---------- Public variables ----------
// Raw accel
volatile int16_t imu_ax_raw = 0;
volatile int16_t imu_ay_raw = 0;
volatile int16_t imu_az_raw = 0;
static   int16_t imu_gz_bias_raw = 0;

// Raw gyro
volatile int16_t imu_gx_raw = 0;
volatile int16_t imu_gy_raw = 0;
volatile int16_t imu_gz_raw = 0;

// Accel in m/s^2
volatile float imu_ax_mps2 = 0.0f;
volatile float imu_ay_mps2 = 0.0f;
volatile float imu_az_mps2 = 0.0f;

// Magnitude of accel vector
volatile float imu_accel_total = 0.0f;

// Yaw (deg)
volatile float imu_yaw_deg = 0.0f;


// ---------- Internal helpers ----------
static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start((MPU_ADDR << 1) | 0);   // write
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

// 🔥 MOVE THIS ABOVE imu_calibrate_gyro SO IT'S KNOWN BEFORE USE
static void mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    // Set register address
    i2c_start((MPU_ADDR << 1) | 0);   // write
    i2c_write(reg);

    // Repeated start, now read
    i2c_start((MPU_ADDR << 1) | 1);   // read

    for (uint8_t i = 0; i < len; i++)
    {
        if (i < (len - 1))
            buf[i] = i2c_readAck();
        else
            buf[i] = i2c_readNak();
    }

    i2c_stop();
}

// ---------- Gyro calibration ----------
void imu_calibrate_gyro(void)
{
    int32_t sum = 0;
    const uint16_t samples = 200;   // 200 samples

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t d[14];
        mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

        int16_t gz_raw = (int16_t)((d[12] << 8) | d[13]);
        sum += gz_raw;

        _delay_ms(10);  // 10 ms between samples
    }

    imu_gz_bias_raw = (int16_t)(sum / samples);
}


// ---------- Public functions ----------
void imu_init(void)
{
    // Init I2C
    i2c_init();

    // Wake MPU-6050 (clear sleep bit)
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    _delay_ms(100);

    // Sample rate: 1kHz / (7+1) = 125 Hz
    mpu_write_reg(REG_SMPLRT_DIV, 0x07);

    // CONFIG: DLPF ~44 Hz
    mpu_write_reg(REG_CONFIG, 0x03);

    // GYRO_CONFIG: ±250 dps
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);

    // ACCEL_CONFIG: ±2g
    mpu_write_reg(REG_ACCEL_CONFIG, 0x00);

    imu_reset_yaw();
}

void imu_reset_yaw(void)
{
    imu_yaw_deg = 0.0f;
}

void imu_update(void)
{
    uint8_t d[14];

    // Read 14 bytes:
    // AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
    // TEMP_H, TEMP_L,
    // GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
    mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

    // ------------ Raw values ------------
    imu_ax_raw = (int16_t)((d[0] << 8) | d[1]);
    imu_ay_raw = (int16_t)((d[2] << 8) | d[3]);
    imu_az_raw = (int16_t)((d[4] << 8) | d[5]);

    imu_gx_raw = (int16_t)((d[8]  << 8) | d[9]);
    imu_gy_raw = (int16_t)((d[10] << 8) | d[11]);
    imu_gz_raw = (int16_t)((d[12] << 8) | d[13]);

    // ------------ Accel in m/s^2 ------------
    imu_ax_mps2 = ((float)imu_ax_raw / ACCEL_SENS) * 9.81f;
    imu_ay_mps2 = ((float)imu_ay_raw / ACCEL_SENS) * 9.81f;
    imu_az_mps2 = ((float)imu_az_raw / ACCEL_SENS) * 9.81f;

    // Total acceleration magnitude
    imu_accel_total = sqrtf(
        imu_ax_mps2 * imu_ax_mps2 +
        imu_ay_mps2 * imu_ay_mps2 +
        imu_az_mps2 * imu_az_mps2
    );

    // ------------ Yaw (degrees) ------------
    // Convert raw gyro Z to deg/s and integrate
    float gz_dps = (float)(imu_gz_raw - imu_gz_bias_raw) / GYRO_SENS;
    imu_yaw_deg += gz_dps * IMU_DT;
}
