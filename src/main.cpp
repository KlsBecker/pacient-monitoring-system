
#include <Arduino.h>
#include <Wire.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>

#ifdef I2C_BUFFER_LENGTH
#undef I2C_BUFFER_LENGTH
#endif
#define I2C_BUFFER_LENGTH 128

// Configurações GPIO
#define ALARM_LED_GPIO GPIO_NUM_14
#define RESET_BUTTON_GPIO GPIO_NUM_23

// Configurações Wi-Fi e MQTT
// #define WIFI_SSID "iPhone de Klaus"
// #define WIFI_PASSWORD "batebola"
#define WIFI_SSID "CASA URUGUAI OI FIBRA "
#define WIFI_PASSWORD "santaluiza#361"
#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_TOPIC_HEARTRATE "patient_monitoring_system_aafk/heart_rate"
#define MQTT_TOPIC_FALL "patient_monitoring_system_aafk/fall"
#define MQTT_TOPIC_AGITATION "patient_monitoring_system_aafk/agitation"

// Thresholds
#define AGITATION_THRESHOLD_GYRO 200.0
#define FALL_THRESHOLD_GYRO 50.0
#define AGITATION_DURATION_MS 3000
#define STABILITY_DURATION_MS 2000

// Variáveis globais
WiFiClient espClient;
PubSubClient mqttClient(espClient);
MAX30105 particleSensor;
MPU6050 mpu;
QueueHandle_t alarmQueue;
QueueHandle_t resetButtonQueue;
SemaphoreHandle_t stateMutex;

bool agitationInProgress = false;
uint32_t agitationStartTime = 0;

bool fallInProgress = false;
uint32_t fallStartTime = 0;

bool fallDetected = false;  // Estado persistente de queda
bool agitationDetected = false;  // Estado persistente de agitação

// Inicializa Wi-Fi
void initWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Wi-Fi conectado!");
}

// Inicializa MQTT
void initMQTT() {
    mqttClient.setServer(MQTT_BROKER, 1883);
    while (!mqttClient.connected()) {
        Serial.println("Conectando ao broker MQTT...");
        if (mqttClient.connect("ESP32Client_aafk")) {
            Serial.println("Conectado ao broker MQTT!");
        } else {
            delay(1000);
        }
    }
}

// Inicializa sensores
void initSensors() {
    if (!particleSensor.begin()) {
        Serial.println("MAX3010x não encontrado!");
        while (1);
    }
    particleSensor.setup();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 não encontrado!");
        while (1);
    }
}

// Detecta agitação com base na magnitude do giroscópio
bool detectAgitation() {
    int16_t gyroX, gyroY, gyroZ;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);

    float gyroMagnitude = sqrt(pow(gyroX / 131.0, 2) +
                               pow(gyroY / 131.0, 2) +
                               pow(gyroZ / 131.0, 2));

    if (gyroMagnitude > AGITATION_THRESHOLD_GYRO) {
        if (!agitationInProgress) {
            agitationInProgress = true;
            agitationStartTime = millis();
        } else if ((millis() - agitationStartTime) >= AGITATION_DURATION_MS) {
            agitationInProgress = false;
            return true;
        }
    } else {
        agitationInProgress = false;
        agitationStartTime = 0;
    }
    return false;
}

// Detecta queda com base na magnitude do giroscópio
bool detectFall() {
    int16_t gyroX, gyroY, gyroZ;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);

    float gyroMagnitude = sqrt(pow(gyroX / 131.0, 2) +
                               pow(gyroY / 131.0, 2) +
                               pow(gyroZ / 131.0, 2));

    if (gyroMagnitude > AGITATION_THRESHOLD_GYRO && !agitationDetected) {
        fallInProgress = true;
        fallStartTime = 0;
        return false;
    }

    if (fallInProgress) {
        if (gyroMagnitude < FALL_THRESHOLD_GYRO) {
            if (fallStartTime == 0) {
                fallStartTime = millis();
            } else if ((millis() - fallStartTime) >= STABILITY_DURATION_MS) {
                fallInProgress = false;
                return true;
            }
        } else {
            fallStartTime = 0;
        }
    }
    return false;
}

// Tarefa de monitoramento de BPM e SpO2
void taskMonitorHeartRate(void *param) {
    uint32_t irBuffer[100], redBuffer[100];
    int32_t spo2 = 0, heartRate = 0;
    int8_t spo2Valid = 0, hrValid = 0;

    while (true) {
        for (int i = 0; i < 100; i++) {
            while (!particleSensor.check());
            irBuffer[i] = particleSensor.getIR();
            redBuffer[i] = particleSensor.getRed();
        }

        maxim_heart_rate_and_oxygen_saturation(
            irBuffer, 100,
            redBuffer,
            &spo2, &spo2Valid,
            &heartRate, &hrValid
        );

        if (spo2Valid && hrValid) {
            char message[50];
            snprintf(message, 50, "%d", heartRate);
            bool result = mqttClient.publish(MQTT_TOPIC_HEARTRATE, message);
            Serial.print("BPM: ");
            Serial.println(message);
        } else {
            Serial.println("Dados inválidos ou insuficientes para cálculo.");
            mqttClient.publish(MQTT_TOPIC_HEARTRATE, "0");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Tarefa de monitoramento de movimento
void taskMonitorMovement(void *param) {
    while (true) {
        // Detecta queda e agitação
        bool fallen = detectFall();
        bool agitaded = detectAgitation();

        // Atualiza estados persistentes
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        if (fallen) {
            fallDetected = true;
        }
        if (agitaded) {
            agitationDetected = true;
        }

        // Publica os estados persistentes no MQTT
        mqttClient.publish(MQTT_TOPIC_FALL, fallDetected ? "true" : "false");
        mqttClient.publish(MQTT_TOPIC_AGITATION, agitationDetected ? "true" : "false");
        xSemaphoreGive(stateMutex);

        // Ativa o alarme se houver queda ou agitação
        if (fallDetected || agitationDetected) {
            bool alarmTriggered = true;
            xQueueSend(alarmQueue, &alarmTriggered, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Tarefa de gerenciamento de alarmes
void taskAlarmHandler(void *param) {
    bool alarmTriggered;

    while (true) {
        if (xQueueReceive(alarmQueue, &alarmTriggered, portMAX_DELAY)) {
            gpio_set_level(ALARM_LED_GPIO, alarmTriggered ? 1 : 0);
        }
    }
}

void taskResetButtonHandler(void *param) {
    while (true) {
        // Verifica o estado do botão
        if (digitalRead(RESET_BUTTON_GPIO) == LOW) { // Botão pressionado
            delay(50); // Debounce
            if (digitalRead(RESET_BUTTON_GPIO) == LOW) {
                Serial.println("Botão de reset pressionado!");

                // Redefine os estados de queda e agitação
                xSemaphoreTake(stateMutex, portMAX_DELAY);
                fallDetected = false;
                agitationDetected = false;
                xSemaphoreGive(stateMutex);

                // Desativa o alarme
                gpio_set_level(ALARM_LED_GPIO, 0);

                // Publica os estados redefinidos no MQTT
                mqttClient.publish(MQTT_TOPIC_FALL, "false");
                mqttClient.publish(MQTT_TOPIC_AGITATION, "false");

                // Aguarda enquanto o botão estiver pressionado
                while (digitalRead(RESET_BUTTON_GPIO) == LOW) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Verificação periódica do botão
    }
}

// Configuração inicial
void setup() {
    Serial.begin(115200);

    initWiFi();
    initMQTT();
    initSensors();

    gpio_reset_pin(ALARM_LED_GPIO);
    gpio_set_direction(ALARM_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALARM_LED_GPIO, 0);

    pinMode(RESET_BUTTON_GPIO, INPUT_PULLUP);

    alarmQueue = xQueueCreate(10, sizeof(bool));
    resetButtonQueue = xQueueCreate(10, sizeof(bool));

    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {   
      Serial.println("Erro ao criar mutex!");
      while (1);
    }

    xTaskCreate(taskMonitorHeartRate, "HeartRate", 4096, NULL, 1, NULL);
    xTaskCreate(taskMonitorMovement, "Movement", 4096, NULL, 1, NULL);
    xTaskCreate(taskAlarmHandler, "AlarmHandler", 4096, NULL, 1, NULL);
    xTaskCreate(taskResetButtonHandler, "ResetHandler", 4096, NULL, 1, NULL);
}

void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.println("Tentando conectar ao broker MQTT...");
        if (mqttClient.connect("ESP32Client_aafk")) {
            Serial.println("MQTT conectado!");
        } else {
            delay(1000); // Aguarde 1 segundo antes de tentar novamente
        }
    }
}


// Loop principal
void loop() {
    mqttClient.loop();

    if (!mqttClient.connected()) {
      Serial.println("MQTT desconectado, tentando reconectar...");
      reconnectMQTT();
    }
  
}
