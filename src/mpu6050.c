#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h" // Adicione esta linha para usar esp_timer_get_time


static const char *TAG = "MPU6050";

// Inicializa o sensor MPU6050
esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    uint8_t data[] = {0x6B, 0x00}; // Configura o sensor para modo ativo
    esp_err_t ret = i2c_master_write_to_device(i2c_num, MPU6050_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 inicializado com sucesso");
    } else {
        ESP_LOGE(TAG, "Falha ao inicializar MPU6050");
    }
    return ret;
}

// Lê dados de aceleração e giroscópio do MPU6050
esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data) {
    uint8_t raw_data[14];
    esp_err_t ret = i2c_master_write_read_device(i2c_num, MPU6050_ADDR, (uint8_t[]){0x3B}, 1, raw_data, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao ler dados do MPU6050");
        return ret;
    }

    // Converte os dados brutos para unidades de aceleração (g) e giroscópio (graus/segundo)
    data->accel_x = ((int16_t)(raw_data[0] << 8 | raw_data[1])) / 16384.0;
    data->accel_y = ((int16_t)(raw_data[2] << 8 | raw_data[3])) / 16384.0;
    data->accel_z = ((int16_t)(raw_data[4] << 8 | raw_data[5])) / 16384.0;
    data->gyro_x = ((int16_t)(raw_data[8] << 8 | raw_data[9])) / 131.0;
    data->gyro_y = ((int16_t)(raw_data[10] << 8 | raw_data[11])) / 131.0;
    data->gyro_z = ((int16_t)(raw_data[12] << 8 | raw_data[13])) / 131.0;

    return ESP_OK;
}

// Função para detectar agitação usando o giroscópio
bool detect_agitation(mpu6050_data_t *data) {
    float gyro_magnitude = sqrt(data->gyro_x * data->gyro_x + data->gyro_y * data->gyro_y + data->gyro_z * data->gyro_z);
    static bool agitation_in_progress = false;
    static uint32_t agitation_start_time = 0;

    if (gyro_magnitude > AGITATION_THRESHOLD_GYRO) {
        if (!agitation_in_progress) {
            agitation_in_progress = true;
            agitation_start_time = esp_timer_get_time() / 1000;
        } else {
            uint32_t elapsed_time = (esp_timer_get_time() / 1000) - agitation_start_time;
            if (elapsed_time >= AGITATION_DURATION_MS) {
                agitation_in_progress = false;
                return true;
            }
        }
    } else {
        agitation_in_progress = false;
        agitation_start_time = 0;
    }
    return false;
}

// Função para detectar queda usando o giroscópio
bool detect_fall(mpu6050_data_t *data) {
    float gyro_magnitude = sqrt(data->gyro_x * data->gyro_x + data->gyro_y * data->gyro_y + data->gyro_z * data->gyro_z);
    static bool fall_in_progress = false;
    static uint32_t fall_start_time = 0;

    if (gyro_magnitude > AGITATION_THRESHOLD_GYRO) {
        fall_in_progress = true;
        fall_start_time = 0;
        return false;
    }

    if (fall_in_progress) {
        if (gyro_magnitude < FALL_THRESHOLD_GYRO) {
            if (fall_start_time == 0) {
                fall_start_time = esp_timer_get_time() / 1000;
            } else {
                uint32_t elapsed_time = (esp_timer_get_time() / 1000) - fall_start_time;
                if (elapsed_time >= STABILITY_DURATION_MS) {
                    fall_in_progress = false;
                    return true;
                }
            }
        } else {
            fall_start_time = 0;
        }
    }
    return false;
}
