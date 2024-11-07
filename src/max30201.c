#include "max30201.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"  // Para trabalhar com o tempo em milissegundos

static const char *TAG = "MAX30201";

// Definições para a configuração do I2C
#define I2C_MASTER_NUM I2C_NUM_0       // Canal I2C que estamos usando
#define I2C_MASTER_SCL_IO 22           // Pino SCL
#define I2C_MASTER_SDA_IO 21           // Pino SDA
#define I2C_MASTER_FREQ_HZ 100000      // Frequência I2C

// Função para inicializar o I2C (internamente usada pelo módulo)
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Função para configurar o MAX30201
esp_err_t max30201_init(void) {
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar o I2C");
        return ret;
    }

    // Configuração básica do sensor, ajuste conforme necessário
    uint8_t config_data[] = {0x09, 0x03};  // Exemplo: Configurar modo do sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30201_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, config_data, sizeof(config_data), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAX30201 inicializado com sucesso");
    } else {
        ESP_LOGE(TAG, "Erro ao configurar o MAX30201");
    }
    return ret;
}

// Função para ler dados do MAX30201
int16_t max30201_read_data(void) {
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30201_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        int16_t heart_rate = (data[0] << 8) | data[1];
        return heart_rate;
    } else {
        ESP_LOGE(TAG, "Erro ao ler dados do sensor");
        return -1;
    }
}
