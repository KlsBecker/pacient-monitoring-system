#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"  // Para trabalhar com o tempo em milissegundos
#include "driver/i2c.h"
#include <math.h>
#include "esp_log.h"

// Variáveis de estado para detecção
static bool agitation_in_progress = false;
static uint32_t agitation_start_time = 0;

static bool fall_in_progress = false;
static uint32_t fall_start_time = 0;

static const char *TAG = "MPU6050";

// Inicializa o MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    uint8_t data[] = {MPU6050_PWR_MGMT_1, 0x00};  // Configura o sensor para modo ativo
    return i2c_master_write_to_device(i2c_num, MPU6050_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

// Lê dados de aceleração e giroscópio
esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data) {
    uint8_t raw_data[14];
    esp_err_t ret = i2c_master_write_read_device(i2c_num, MPU6050_ADDR, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, raw_data, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    data->accel_x = ((int16_t)(raw_data[0] << 8 | raw_data[1])) / 16384.0;
    data->accel_y = ((int16_t)(raw_data[2] << 8 | raw_data[3])) / 16384.0;
    data->accel_z = ((int16_t)(raw_data[4] << 8 | raw_data[5])) / 16384.0;
    data->gyro_x = ((int16_t)(raw_data[8] << 8 | raw_data[9])) / 131.0;
    data->gyro_y = ((int16_t)(raw_data[10] << 8 | raw_data[11])) / 131.0;
    data->gyro_z = ((int16_t)(raw_data[12] << 8 | raw_data[13])) / 131.0;

    return ESP_OK;
}

// Função para detectar agitação com base na velocidade angular do giroscópio
bool detect_agitation(mpu6050_data_t *data) {
    float gyro_magnitude = sqrt(data->gyro_x * data->gyro_x + data->gyro_y * data->gyro_y + data->gyro_z * data->gyro_z);

    // Detecta movimento brusco contínuo
    if (gyro_magnitude > AGITATION_THRESHOLD_GYRO) {
        if (!agitation_in_progress) {
            agitation_in_progress = true;
            agitation_start_time = esp_timer_get_time() / 1000;  // Tempo em milissegundos
        } else {
            uint32_t elapsed_time = (esp_timer_get_time() / 1000) - agitation_start_time;
            if (elapsed_time >= AGITATION_DURATION_MS) {
                agitation_in_progress = false;
                return true;  // Confirma a agitação
            }
        }
    } else {
        // Reseta o estado se o movimento brusco para antes dos 3 segundos
        agitation_in_progress = false;
        agitation_start_time = 0;
    }
    return false;
}

// Função para detectar queda com base na velocidade angular do giroscópio
bool detect_fall(mpu6050_data_t *data) {
    float gyro_magnitude = sqrt(data->gyro_x * data->gyro_x + data->gyro_y * data->gyro_y + data->gyro_z * data->gyro_z);

    // Detecta agitação inicial
    if (gyro_magnitude > AGITATION_THRESHOLD_GYRO) {
        fall_in_progress = true;
        fall_start_time = 0;  // Reseta o tempo de estabilidade
        return false;
    }

    // Monitorar estabilidade após a agitação
    if (fall_in_progress) {
        if (gyro_magnitude < FALL_THRESHOLD_GYRO) {
            if (fall_start_time == 0) {
                fall_start_time = esp_timer_get_time() / 1000;  // Em milissegundos
            } else {
                uint32_t elapsed_time = (esp_timer_get_time() / 1000) - fall_start_time;
                if (elapsed_time >= STABILITY_DURATION_MS) {
                    fall_in_progress = false;
                    return true;  // Confirma a queda
                }
            }
        } else {
            // Reseta o estado de estabilidade se detecta movimento
            fall_start_time = 0;
        }
    }
    return false;
}
