#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_data_t;

esp_err_t mpu6050_init(i2c_port_t i2c_num);
esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data);
bool detect_fall(mpu6050_data_t *data);
bool detect_agitation(mpu6050_data_t *data);

#endif // MPU6050_H
