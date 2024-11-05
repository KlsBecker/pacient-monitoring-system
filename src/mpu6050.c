#include "mpu6050.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h"  // Para medir o tempo em milissegundos
#define AGITATION_FALL_THRESHOLD 2.0         // Threshold de aceleração para detectar agitação brusca
#define STABILITY_THRESHOLD 1        // Threshold de estabilidade para baixa movimentação
#define STABILITY_DURATION_MS 2000      // Duração de estabilidade para confirmar a queda (em ms)
#define AGITATION_THRESHOLD 1.5          // Threshold de aceleração para detectar agitação brusca
#define AGITATION_DURATION_MS 1000        // Duração mínima para confirmar agitação (3 segundos)

// Variáveis de estado para detecção de agitação
static bool agitation_in_progress = false;
static uint32_t agitation_start_time = 0;

// Variáveis de estado para o algoritmo de detecção de queda
static bool agitation_detected = false;
static uint32_t stability_start_time = 0;


static const char *TAG = "MPU6050";

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    uint8_t data[] = {MPU6050_PWR_MGMT_1, 0x00};
    esp_err_t ret = i2c_master_write_to_device(i2c_num, MPU6050_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 inicializado com sucesso");
    } else {
        ESP_LOGE(TAG, "Falha ao inicializar o MPU6050");
    }
    return ret;
}

esp_err_t mpu6050_read_accel_gyro(i2c_port_t i2c_num, mpu6050_data_t *data) {
    uint8_t raw_data[14];
    esp_err_t ret = i2c_master_write_read_device(i2c_num, MPU6050_ADDR, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, 1, raw_data, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler dados do MPU6050");
        return ret;
    }

    data->accel_x = ((int16_t)(raw_data[0] << 8 | raw_data[1])) / 16384.0;
    data->accel_y = ((int16_t)(raw_data[2] << 8 | raw_data[3])) / 16384.0;
    data->accel_z = ((int16_t)(raw_data[4] << 8 | raw_data[5])) / 16384.0;
    data->gyro_x = ((int16_t)(raw_data[8] << 8 | raw_data[9])) / 131.0;
    data->gyro_y = ((int16_t)(raw_data[10] << 8 | raw_data[11])) / 131.0;
    data->gyro_z = ((int16_t)(raw_data[12] << 8 | raw_data[13])) / 131.0;

    return ESP_OK;
}

// Função para detectar queda com base no eixo Z da aceleração
bool detect_fall(mpu6050_data_t *data) {
    // Calcula a magnitude da aceleração para detectar agitação
    float magnitude = sqrt(data->accel_x * data->accel_x + data->accel_y * data->accel_y + data->accel_z * data->accel_z);
    ESP_LOGI(TAG, "magnitude: %f", magnitude);

    // 1. Detectar uma agitação brusca
    if (magnitude > AGITATION_FALL_THRESHOLD) {
        // Marca que uma agitação brusca foi detectada
        agitation_detected = true;
        stability_start_time = 0;  // Reseta o tempo de estabilidade
        return false;              // Ainda não confirma a queda
    }

    // 2. Monitorar estabilidade após a agitação
    if (agitation_detected) {
        // Se a magnitude está abaixo do threshold de estabilidade
        if (magnitude < STABILITY_THRESHOLD) {
            if (stability_start_time == 0) {
                // Inicia o contador de tempo de estabilidade
                stability_start_time = esp_timer_get_time() / 1000;  // Em milissegundos
            } else {
                // Calcula o tempo decorrido em estado de estabilidade
                uint32_t elapsed_time = (esp_timer_get_time() / 1000) - stability_start_time;
                if (elapsed_time >= STABILITY_DURATION_MS) {
                    // Se a estabilidade persiste por 3 segundos, confirma a queda
                    agitation_detected = false;  // Reseta o estado de agitação
                    return true;                 // Confirma a queda
                }
            }
        } else {
            // Se o sensor detecta movimento novamente, reseta o estado de estabilidade
            stability_start_time = 0;
        }
    }

    return false;  // Nenhuma queda detectada ainda
}

// Função para detectar agitação com base na magnitude da aceleração
bool detect_agitation(mpu6050_data_t *data) {
    // Calcula a magnitude da aceleração para verificar o movimento brusco
    float magnitude = sqrt(data->accel_x * data->accel_x + data->accel_y * data->accel_y + data->accel_z * data->accel_z);

    // 1. Detecta se há um movimento brusco
    if (magnitude > AGITATION_THRESHOLD) {
        // Inicia o temporizador se o movimento brusco começa
        if (!agitation_in_progress) {
            agitation_in_progress = true;
            agitation_start_time = esp_timer_get_time() / 1000;  // Tempo em milissegundos
        } else {
            // Verifica o tempo decorrido em movimento brusco
            uint32_t elapsed_time = (esp_timer_get_time() / 1000) - agitation_start_time;
            if (elapsed_time >= AGITATION_DURATION_MS) {
                // Se o movimento brusco persiste por 3 segundos, confirma a agitação
                agitation_in_progress = false;  // Reseta o estado de agitação
                return true;                    // Confirma a agitação
            }
        }
    } else {
        // Reseta o estado se o movimento brusco para antes dos 3 segundos
        agitation_in_progress = false;
        agitation_start_time = 0;
    }

    return false;  // Nenhuma agitação detectada ainda
}