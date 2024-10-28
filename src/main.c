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

// Configurações principais
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"
#define MQTT_URI "mqtt://mqtt_server_ip"
#define MQTT_TOPIC_ALERT "patient/alert"

// Limites para frequência cardíaca
#define HEART_RATE_MIN 60
#define HEART_RATE_MAX 120

// Thresholds para detecção de movimento e estabilidade
#define FALL_THRESHOLD 2.5     // Valor de aceleração G para detectar pico de queda
#define STABILITY_THRESHOLD 0.3 // Estabilidade de aceleração para considerar imóvel
#define AGITATION_THRESHOLD 0.5 // Threshold de variação para detectar agitação
#define STABILITY_DURATION_MS 3000  // Tempo de estabilidade (em ms) para confirmar uma queda

// Configurações do botão de reset (GPIO)
#define RESET_BUTTON_GPIO GPIO_NUM_0  // Usando GPIO0 como exemplo

// Configurações de FreeRTOS
#define TASK_STACK_SIZE 4096
#define HEART_MONITOR_INTERVAL_MS 10000  // 10 segundos
#define MOVEMENT_MONITOR_INTERVAL_MS 500  // 500 ms para monitoramento frequente
#define ALARM_CHECK_INTERVAL_MS 1000  // Verifica alarmes a cada 1 segundo

// Definição de tipos para fila de alarme
typedef enum {
    ALARM_TRIGGERED,   // Alarme foi ativado
    ALARM_RESET        // Alarme foi resetado
} alarm_event_t;

// Variáveis globais
static const char *TAG = "HeartSafeSystem";
int heart_rate = 0;  // Variável simulada para frequência cardíaca
esp_mqtt_client_handle_t mqtt_client;
QueueHandle_t alarm_queue;
QueueHandle_t reset_button_queue;

// Instanciando o objeto MPU6050
MPU6050 mpu(Wire);

// Funções de conexão Wi-Fi e MQTT
void wifi_init();
void mqtt_init();
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

// Funções para simular sensores
int read_heart_rate();  // Simula leitura do sensor de frequência cardíaca

// Funções principais das tasks
void task_monitor_heart_rate(void *param);
void task_monitor_movement(void *param);
void task_alarm_handler(void *param);
void task_reset_button_handler(void *param);

// Função para inicializar botão de reset
void reset_button_init();

// Prototipo da função ISR do botão
static void IRAM_ATTR gpio_isr_handler(void *arg);

// Função para detecção de queda
void detect_fall_and_agitation();

void app_main() {
    // Inicializar armazenamento NVS para Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar Wi-Fi e MQTT
    wifi_init();
    mqtt_init();

    // Inicializar MPU6050
    Wire.begin();
    mpu.begin();
    mpu.calcGyroOffsets(true);  // Calibrando o giroscópio

    // Inicializar botão de reset
    reset_button_init();

    // Criar fila de eventos de alarme
    alarm_queue = xQueueCreate(10, sizeof(alarm_event_t));

    // Criar tasks para monitorar sensores e lidar com alarmes
    xTaskCreate(task_monitor_heart_rate, "MonitorHeartRate", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_monitor_movement, "MonitorMovement", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_alarm_handler, "AlarmHandler", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_reset_button_handler, "ResetButtonHandler", TASK_STACK_SIZE, NULL, 1, NULL);
}

/**
 * Inicializa a conexão Wi-Fi
 */
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

/**
 * Inicializa a conexão MQTT e registra o handler de eventos
 */
void mqtt_init() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}

/**
 * Handler para eventos do MQTT
 */
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

/**
 * Task para monitorar a frequência cardíaca a cada 10 segundos
 */
void task_monitor_heart_rate(void *param) {
    while (true) {
        heart_rate = read_heart_rate();
        ESP_LOGI(TAG, "Heart Rate: %d bpm", heart_rate);

        if (heart_rate < HEART_RATE_MIN || heart_rate > HEART_RATE_MAX) {
            alarm_event_t event = ALARM_TRIGGERED;
            xQueueSend(alarm_queue, &event, portMAX_DELAY);  // Envia evento de alarme ativado para a fila
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "Heart rate out of range!", 0, 1, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(HEART_MONITOR_INTERVAL_MS));
    }
}

/**
 * Task para monitorar movimento a cada 500ms (queda/agitação)
 */
void task_monitor_movement(void *param) {
    while (true) {
        // Atualizar os dados do MPU6050
        mpu.update();
        detect_fall_and_agitation();
        vTaskDelay(pdMS_TO_TICKS(MOVEMENT_MONITOR_INTERVAL_MS));
    }
}

/**
 * Task que valida o estado de alarme e pode acionar alarmes locais
 */
void task_alarm_handler(void *param) {
    alarm_event_t event;
    while (true) {
        if (xQueueReceive(alarm_queue, &event, portMAX_DELAY)) {
            switch (event) {
                case ALARM_TRIGGERED:
                    ESP_LOGW(TAG, "Alarm triggered! Taking action...");
                    // Aqui pode acionar um buzzer ou outro dispositivo de alarme
                    break;
                case ALARM_RESET:
                    ESP_LOGI(TAG, "Alarm reset!");
                    // Resetar o estado do alarme
                    break;
            }
        }
    }
}

/**
 * Task que lida com o botão de reset para desligar o alarme
 */
void task_reset_button_handler(void *param) {
    int button_state = 0;
    while (true) {
        if (xQueueReceive(reset_button_queue, &button_state, portMAX_DELAY)) {
            if (button_state == 1) {
                alarm_event_t event = ALARM_RESET;
                xQueueSend(alarm_queue, &event, portMAX_DELAY);  // Envia evento de reset do alarme para a fila
            }
        }
    }
}

/**
 * Inicializa o botão de reset
 */
void reset_button_init() {
    reset_button_queue = xQueueCreate(10, sizeof(int));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Interrupção na borda de subida
    };
    gpio_config(&io_conf);

    // Criar uma task que monitora o estado do botão
    gpio_install_isr_service(0);
    gpio_isr_handler_add(RESET_BUTTON_GPIO, gpio_isr_handler, (void *)RESET_BUTTON_GPIO);
}

/**
 * Simula a leitura do sensor de frequência cardíaca (MAX30102)
 */
int read_heart_rate() {
    return (rand() % (HEART_RATE_MAX + 20));  // Valor entre 0 e 120
}

/**
 * Detecta queda e agitação com base nos valores do MPU6050
 */
void detect_fall_and_agitation() {
    float accel = mpu.getAccZ();
    float gyro = mpu.getGyroZ();
    static bool fall_detected = false;
    static TickType_t fall_start_time = 0;

    // Detecção de queda: pico seguido de estabilidade
    if (accel > FALL_THRESHOLD) {
        ESP_LOGI(TAG, "Possible fall detected, checking stability...");
        fall_detected = true;
        fall_start_time = xTaskGetTickCount();
    }

    // Após o pico de aceleração, verifica se há estabilidade
    if (fall_detected) {
        if (fabs(accel) < STABILITY_THRESHOLD && fabs(gyro) < STABILITY_THRESHOLD) {
            if (xTaskGetTickCount() - fall_start_time > pdMS_TO_TICKS(STABILITY_DURATION_MS)) {
                ESP_LOGI(TAG, "Fall confirmed, patient is stable.");
                fall_detected = false;
                alarm_event_t event = ALARM_TRIGGERED;
                xQueueSend(alarm_queue, &event, portMAX_DELAY);  // Envia evento de queda para a fila
                esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "Fall detected!", 0, 1, 0);
            }
        } else {
            // Se os valores estão flutuando, ainda não consideramos uma queda completa
            ESP_LOGI(TAG, "Stability not reached after fall detection.");
        }
    }

    // Detecção de agitação: se houver muita variação nos dados
    if (fabs(accel) > AGITATION_THRESHOLD || fabs(gyro) > AGITATION_THRESHOLD) {
        ESP_LOGI(TAG, "Agitated movement detected!");
        alarm_event_t event = ALARM_TRIGGERED;
        xQueueSend(alarm_queue, &event, portMAX_DELAY);  // Envia evento de agitação para a fila
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ALERT, "Agitated movement detected!", 0, 1, 0);
    }
}

/**
 * Handler de interrupção para o botão de reset
 */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    int pin = (int)arg;
    int state = gpio_get_level(pin);
    xQueueSendFromISR(reset_button_queue, &state, NULL);
}