#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>

/* ============================================================
 *  IMU Sensör (MPU6050) - Teensy 4.1
 *  Wire (I2C Bus 0) üzerinden: Pin 18 (SDA), Pin 19 (SCL)
 *
 *  Complementary Filter ile yaw/pitch/roll hesaplama
 *  ±2000°/s gyro, ±2g ivme ölçer konfigürasyonu
 * ============================================================ */

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
} IMU_Data;

bool imu_init(void);
void imu_calibrate(IMU_Data *data, uint16_t samples);
void imu_read(IMU_Data *data, float dt);

#endif 
