#include "imu_sensor.h"
#include "config.h"
#include "dsp_filter.h"
#include <Wire.h>
#include <math.h>

#define RAD_TO_DEG  (180.0f / 3.14159265f)

static BiquadFilter s_gz_notch;
static BiquadFilter s_gz_lpf;

static bool write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static uint8_t read_reg(uint8_t reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.read();
}

bool imu_init(void)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    // 1 kHz kontrol döngüsünde 120 Hz itici gövde rezonansı için çentik (notch)
    // ve 35 Hz 2. derece Butterworth alçak geçiren filtre kaskadı
    s_gz_notch.configure_notch(1000.0f, 120.0f, 4.0f);
    s_gz_lpf.configure_lowpass(1000.0f, 35.0f, 0.70710678f);

    uint8_t whoami = read_reg(MPU6050_WHO_AM_I);
    if (whoami != 0x68)
        return false;

    write_reg(MPU6050_PWR_MGMT_1, 0x01);
    delay(50);

    write_reg(MPU6050_SMPLRT_DIV, 0x09);

    write_reg(MPU6050_CONFIG_REG, 0x03);

    write_reg(MPU6050_GYRO_CONFIG, 0x18);

    write_reg(MPU6050_ACCEL_CONFIG, 0x00);

    return true;
}

void imu_calibrate(IMU_Data *data, uint16_t samples)
{
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint8_t buf[14];

    for (uint16_t i = 0; i < samples; i++) {

        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(MPU6050_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);

        for (uint8_t j = 0; j < 14; j++)
            buf[j] = Wire.read();

        int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

        sum_gx += gx * GYRO_SENSITIVITY;
        sum_gy += gy * GYRO_SENSITIVITY;
        sum_gz += gz * GYRO_SENSITIVITY;

        delay(5);
    }

    data->gyro_offset_x = sum_gx / samples;
    data->gyro_offset_y = sum_gy / samples;
    data->gyro_offset_z = sum_gz / samples;

    data->roll = 0.0f;
    data->pitch = 0.0f;
    data->yaw = 0.0f;
    s_gz_notch.reset();
    s_gz_lpf.reset();
}

void imu_read(IMU_Data *data, float dt)
{
    uint8_t buf[14];

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return;

    uint8_t count = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
    if (count < 14) return;

    for (uint8_t i = 0; i < 14; i++)
        buf[i] = Wire.read();

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    data->accel_x = ax_raw * ACCEL_SENSITIVITY;
    data->accel_y = ay_raw * ACCEL_SENSITIVITY;
    data->accel_z = az_raw * ACCEL_SENSITIVITY;

    data->gyro_x = gx_raw * GYRO_SENSITIVITY - data->gyro_offset_x;
    data->gyro_y = gy_raw * GYRO_SENSITIVITY - data->gyro_offset_y;
    float raw_gz = gz_raw * GYRO_SENSITIVITY - data->gyro_offset_z;
    data->gyro_z = s_gz_lpf.step(s_gz_notch.step(raw_gz));

    float accel_roll = atan2f(data->accel_y, data->accel_z) * RAD_TO_DEG;
    float accel_pitch = atan2f(-data->accel_x,
        sqrtf(data->accel_y * data->accel_y +
              data->accel_z * data->accel_z)) * RAD_TO_DEG;

    data->roll = COMP_FILTER_ALPHA * (data->roll + data->gyro_x * dt)
                 + (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    data->pitch = COMP_FILTER_ALPHA * (data->pitch + data->gyro_y * dt)
                  + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;

    data->yaw += data->gyro_z * dt;

    if (data->yaw > 360.0f) data->yaw -= 360.0f;
    if (data->yaw < 0.0f)   data->yaw += 360.0f;
}
