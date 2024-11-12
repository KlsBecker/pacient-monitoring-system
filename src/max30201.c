#include "max30201.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define MAX30201_ADDR 0x50  // Endereço I2C do MAX30201
static const char *TAG = "MAX30201";

// Inicializa o sensor MAX30201
esp_err_t max30201_init(i2c_port_t i2c_num) {
    uint8_t config_data[] = {0x01, 0x00}; // Configuração inicial (exemplo)
    esp_err_t ret = i2c_master_write_to_device(i2c_num, MAX30201_ADDR, config_data, sizeof(config_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAX30201 inicializado com sucesso");
    } else {
        ESP_LOGE(TAG, "Falha ao inicializar MAX30201");
    }
    return ret;
}

// Lê a frequência cardíaca do MAX30201
int max30201_read_data(i2c_port_t i2c_num) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_read_from_device(i2c_num, MAX30201_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        int heart_rate = (data[0] << 8) | data[1];
        return heart_rate;
    } else {
        ESP_LOGE(TAG, "Erro ao ler dados do MAX30201");
        return -1;
    }
}
