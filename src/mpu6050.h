#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include "driver/i2c.h"

// Endereço I2C do MPU6050
#define MPU6050_ADDR 0x68

// Thresholds para detecção de agitação e queda
#define AGITATION_THRESHOLD_GYRO 200.0     // Threshold de velocidade angular para agitação
#define FALL_THRESHOLD_GYRO 50.0           // Threshold de baixa velocidade angular para estabilidade
#define AGITATION_DURATION_MS 3000         // Duração mínima de agitação para confirmar agitação (em ms)
#define STABILITY_DURATION_MS 3000         // Duração mínima de estabilidade para confirmar queda (em ms)

// Estrutura para armazenar dados do sensor MPU6050
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_data_t;

// Funções para inicializar e ler dados do sensor MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num);
esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data);

// Funções de detecção de agitação e queda baseadas no giroscópio do MPU6050
bool detect_agitation(mpu6050_data_t *data);
bool detect_fall(mpu6050_data_t *data);

#endif // MPU6050_H
