// imu.c
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "imu.h"
#include "i2c/i2c.h"


#define MPU_ADDR            0x68    

#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B    \


#define ACCEL_SENS          16384.0f
#define GYRO_SENS           131.0f


#define IMU_DT              0.05f


#define GZ_DEADZONE_DPS     0.5f


volatile int16_t imu_ax_raw = 0;
volatile int16_t imu_ay_raw = 0;
volatile int16_t imu_az_raw = 0;


volatile int16_t imu_gx_raw = 0;
volatile int16_t imu_gy_raw = 0;
volatile int16_t imu_gz_raw = 0;


static int16_t imu_gz_bias_raw = 0;


volatile float imu_ax_mps2 = 0.0f;
volatile float imu_ay_mps2 = 0.0f;
volatile float imu_az_mps2 = 0.0f;


volatile float imu_accel_total = 0.0f;


volatile float imu_yaw_deg = 0.0f;




static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start((MPU_ADDR << 1) | 0);  
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

static void mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
   
    i2c_start((MPU_ADDR << 1) | 0);  
    i2c_write(reg);

   
    i2c_start((MPU_ADDR << 1) | 1);   

    for (uint8_t i = 0; i < len; i++)
    {
        if (i < (len - 1))
            buf[i] = i2c_readAck();
        else
            buf[i] = i2c_readNak();
    }

    i2c_stop();
}


void imu_calibrate_gyro(void)
{
    int32_t sum = 0;
    const uint16_t samples = 200;   

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t d[14];
        mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

        int16_t gz_raw = (int16_t)((d[12] << 8) | d[13]);
        sum += gz_raw;

        _delay_ms(10);  
    }

    imu_gz_bias_raw = (int16_t)(sum / samples);
}




void imu_init(void)
{
   
    i2c_init();

   
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    _delay_ms(100);

   
    mpu_write_reg(REG_SMPLRT_DIV, 0x07);

   
    mpu_write_reg(REG_CONFIG, 0x03);

    
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);

    
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

    
    mpu_read_bytes(REG_ACCEL_XOUT_H, d, 14);

    
    imu_ax_raw = (int16_t)((d[0] << 8) | d[1]);
    imu_ay_raw = (int16_t)((d[2] << 8) | d[3]);
    imu_az_raw = (int16_t)((d[4] << 8) | d[5]);

    imu_gx_raw = (int16_t)((d[8]  << 8) | d[9]);
    imu_gy_raw = (int16_t)((d[10] << 8) | d[11]);
    imu_gz_raw = (int16_t)((d[12] << 8) | d[13]);

 
    imu_ax_mps2 = ((float)imu_ax_raw / ACCEL_SENS) * 9.81f;
    imu_ay_mps2 = ((float)imu_ay_raw / ACCEL_SENS) * 9.81f;
    imu_az_mps2 = ((float)imu_az_raw / ACCEL_SENS) * 9.81f;

   
    imu_accel_total = sqrtf(
        imu_ax_mps2 * imu_ax_mps2 +
        imu_ay_mps2 * imu_ay_mps2 +
        imu_az_mps2 * imu_az_mps2
    );

   
    float gz_dps = (float)(imu_gz_raw - imu_gz_bias_raw) / GYRO_SENS;

    
    if (fabsf(gz_dps) < GZ_DEADZONE_DPS) {
        gz_dps = 0.0f;
    }

    imu_yaw_deg += gz_dps * IMU_DT;
}
