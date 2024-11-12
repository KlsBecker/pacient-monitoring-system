#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "mpu6050.h"
#include "max30201.h"
#include "driver/i2c.h"
#include "esp_timer.h" // Para usar esp_timer_get_time

// Definições de constantes
#define TASK_STACK_SIZE 4096
#define HEART_MONITOR_INTERVAL_MS 10000   // Intervalo de monitoramento de frequência cardíaca
#define MOVEMENT_MONITOR_INTERVAL_MS 500  // Intervalo de monitoramento de movimento
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// Configurações do MQTT
#define MQTT_URI                    "mqtt://test.mosquitto.org:1883"
#define MQTT_TOPIC_ROOT             "patient_monitoring_system_aafk"
#define MQTT_TOPIC_HEARTRATE        MQTT_TOPIC_ROOT "/heart_rate"
#define MQTT_TOPIC_FALL             MQTT_TOPIC_ROOT "/fall"
#define MQTT_TOPIC_AGITATION        MQTT_TOPIC_ROOT "/agitation"

// Configurações principais
#define WIFI_SSID "iPhone de Klaus"
#define WIFI_PASSWORD "batebola"

// Configurações do botão de reset e LED de alarme
#define RESET_BUTTON_GPIO GPIO_NUM_23
#define ALARM_LED_GPIO GPIO_NUM_14
#define WIFI_LED_GPIO GPIO_NUM_2

// Definição de tipos para fila de alarme
typedef enum {
    ALARM_TRIGGERED,   // Alarme foi ativado
    ALARM_RESET        // Alarme foi resetado
} alarm_event_t;

// Declarações de funções
void led_wifi_init();
void reset_button_init();
static void IRAM_ATTR gpio_isr_handler(void *arg);

// Variáveis globais
static const char *TAG = "PatientMonitoringSystem";
int heart_rate = 0;
bool fallen = false;
bool agitaded = false;
esp_mqtt_client_handle_t mqtt_client;
QueueHandle_t alarm_queue;
QueueHandle_t reset_button_queue;
bool wifi_connected = false;

// Funções de conexão Wi-Fi e MQTT
void wifi_init();
void mqtt_init();
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

// Funções principais das tasks
void task_monitor_heart_rate(void *param);
void task_monitor_movement(void *param);
void task_alarm_handler(void *param);
void task_reset_button_handler(void *param);
void task_wifi_led_indicator(void *param);

// Função para inicializar o barramento I2C
void i2c_master_init() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode, 0, 0, 0);
}

// Inicializa o LED de alarme
void alarm_led_init() {
    gpio_reset_pin(ALARM_LED_GPIO);
    gpio_set_direction(ALARM_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALARM_LED_GPIO, 0);  // LED desligado inicialmente
}

void app_main() {
    // Inicializa o I2C
    i2c_master_init();

    // Inicializa sensores MPU6050 e MAX30201
    if (mpu6050_init(I2C_MASTER_NUM) == ESP_OK) {
        ESP_LOGI("MPU6050", "Sensor MPU6050 inicializado com sucesso");
    } else {
        ESP_LOGE("MPU6050", "Erro ao inicializar o sensor MPU6050");
    }

    if (max30201_init(I2C_MASTER_NUM) == ESP_OK) {
        ESP_LOGI("MAX30201", "Sensor MAX30201 inicializado com sucesso");
    } else {
        ESP_LOGE("MAX30201", "Erro ao inicializar o sensor MAX30201");
    }

    // Inicializações de Wi-Fi, MQTT, LED e botão de reset
    wifi_init();
    led_wifi_init();
    reset_button_init();
    alarm_led_init();

    // Cria fila de eventos de alarme
    alarm_queue = xQueueCreate(10, sizeof(alarm_event_t));

    // Criação de tasks para monitoramento e alarme
    xTaskCreate(task_monitor_heart_rate, "MonitorHeartRate", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_monitor_movement, "MonitorMovement", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_alarm_handler, "AlarmHandler", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_reset_button_handler, "ResetButtonHandler", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_wifi_led_indicator, "WifiLEDIndicator", TASK_STACK_SIZE, NULL, 1, NULL);
}

void wifi_init() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    esp_wifi_connect();
}

void mqtt_init() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        default:
            ESP_LOGI(TAG, "Other MQTT Event: %" PRId32, event_id);
            break;
    }
}

void task_monitor_heart_rate(void *param) {
    char heart_rate_str[10];
    while (true) {
        heart_rate = max30201_read_data(I2C_MASTER_NUM);
        if (heart_rate >= 0) {
            ESP_LOGI(TAG, "Heart Rate: %d bpm", heart_rate);
            sprintf(heart_rate_str, "%d", heart_rate);
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_HEARTRATE, heart_rate_str, strlen(heart_rate_str), 0, 0);
        } else {
            ESP_LOGW(TAG, "Erro ao ler frequência cardíaca");
        }
        vTaskDelay(pdMS_TO_TICKS(HEART_MONITOR_INTERVAL_MS));
    }
}

void task_monitor_movement(void *param) {
    mpu6050_data_t sensor_data;
    while (true) {
        if (mpu6050_read_accel_gyro(I2C_MASTER_NUM, &sensor_data) == ESP_OK) {
            agitaded |= detect_agitation(&sensor_data);
            ESP_LOGI(TAG, "Agitação: %s", agitaded ? "yes" : "no");
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_AGITATION, agitaded ? "true" : "false", 0, 0, 0);

            fallen |= detect_fall(&sensor_data) && !agitaded;
            ESP_LOGI(TAG, "Queda: %s", fallen ? "yes" : "no");
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_FALL, fallen ? "true" : "false", 0, 0, 0);

            if (fallen || agitaded) {
                alarm_event_t event = ALARM_TRIGGERED;
                xQueueSend(alarm_queue, &event, portMAX_DELAY);
            }

            vTaskDelay(pdMS_TO_TICKS(MOVEMENT_MONITOR_INTERVAL_MS));
        }
    }
}

void task_alarm_handler(void *param) {
    alarm_event_t event;
    while (true) {
        if (xQueueReceive(alarm_queue, &event, portMAX_DELAY)) {
            switch (event) {
                case ALARM_TRIGGERED:
                    ESP_LOGW(TAG, "Alarm triggered! Taking action...");
                    gpio_set_level(ALARM_LED_GPIO, 1);
                    break;
                case ALARM_RESET:
                    ESP_LOGI(TAG, "Alarm reset!");
                    gpio_set_level(ALARM_LED_GPIO, 0);
                    fallen = agitaded = false;
                    break;
            }
        }
    }
}

void task_reset_button_handler(void *param) {
    int button_state = 0;
    while (true) {
        if (xQueueReceive(reset_button_queue, &button_state, portMAX_DELAY)) {
            if (button_state == 1) {
                alarm_event_t event = ALARM_RESET;
                xQueueSend(alarm_queue, &event, portMAX_DELAY);
            }
        }
    }
}

void reset_button_init() {
    reset_button_queue = xQueueCreate(10, sizeof(int));
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RESET_BUTTON_GPIO, gpio_isr_handler, (void *)RESET_BUTTON_GPIO);
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    int pin = (int)arg;
    int state = gpio_get_level(pin);
    xQueueSendFromISR(reset_button_queue, &state, NULL);
}

void led_wifi_init() {
    gpio_reset_pin(WIFI_LED_GPIO);
    gpio_set_direction(WIFI_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(WIFI_LED_GPIO, 0);
}

void task_wifi_led_indicator(void *param) {
    wifi_ap_record_t ap_info;
    while (true) {
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            if (!wifi_connected) {
                wifi_connected = true;
                mqtt_init();
            }
            gpio_set_level(WIFI_LED_GPIO, 1);
        } else {
            wifi_connected = false;
            gpio_set_level(WIFI_LED_GPIO, !gpio_get_level(WIFI_LED_GPIO));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
