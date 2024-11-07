#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// Configurações do sensor
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// Thresholds para detecção de agitação e queda com giroscópio
#define AGITATION_THRESHOLD_GYRO 200.0   // Threshold de velocidade angular para agitação (em graus/seg)
#define FALL_THRESHOLD_GYRO 50.0         // Threshold de baixa velocidade angular para queda (em graus/seg)
#define AGITATION_DURATION_MS 3000       // Duração mínima de agitação para confirmar agitação (3 segundos)
#define STABILITY_DURATION_MS 3000       // Duração mínima de estabilidade para confirmar queda (3 segundos)

// Estrutura para armazenar os dados do sensor
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_data_t;

// Funções para inicializar e ler dados do MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num);
esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data);

// Funções para detecção de agitação e queda usando giroscópio
bool detect_fall(mpu6050_data_t *data);
bool detect_agitation(mpu6050_data_t *data);

#endif // MPU6050_H
