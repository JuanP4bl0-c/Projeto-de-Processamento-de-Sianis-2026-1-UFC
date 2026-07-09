#include "mqtt_bridge.h"
#include "current_monitor.h"

#include <stdio.h>


#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

#include "mqtt_client.h"

#include "nvs_flash.h"
#include "wifi_manager.h"
#include "freertos/FreeRTOS.h"

#include "freertos/queue.h"
#include "freertos/task.h"

#define WIFI_SSID "brisa-2504280"
#define WIFI_PASSWORD "eubcidpn"
#define MQTT_BROKER_URI "mqtt://192.168.0.6:1883"
#define WIFI_AUTH_MODE WIFI_AUTH_WPA2_PSK

// #define WIFI_SSID "LANCHONETE"
// #define WIFI_PASSWORD "09260224"
// #define MQTT_BROKER_URI "mqtt://10.10.220.31:1883"
// #define WIFI_AUTH_MODE WIFI_AUTH_WPA2_PSK

// #define WIFI_SSID "UFC_B4_SL3_2"
// #define WIFI_PASSWORD ""
// #define MQTT_BROKER_URI "mqtt://10.0.105.207:1883"
// #define WIFI_AUTH_MODE WIFI_AUTH_OPEN

#define MQTT_CLIENT_ID "esp32-current-monitor"
#define MQTT_TOPIC "esp32/sinal"

#define WIFI_CONNECT_TIMEOUT_MS 20000
#define MQTT_RETRY_DELAY_MS 1000

static const char *TAG = "MQTT_BRIDGE";
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

static esp_err_t mqtt_bridge_init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado ao broker");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT desconectado");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT publish confirmado, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Erro no MQTT");
            break;
        default:
            break;
    }
}

static esp_err_t mqtt_bridge_start_client(void)
{
    if (s_mqtt_client != NULL) {
        return ESP_OK;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = MQTT_CLIENT_ID,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "Falha ao inicializar cliente MQTT");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt_client));
    return ESP_OK;
}

void mqtt_bridge_task(void *pvParameters)
{
    QueueHandle_t report_queue = (QueueHandle_t)pvParameters;
    current_monitor_report_t report = { 0 };
    char payload[256];

    if (report_queue == NULL) {
        ESP_LOGE(TAG, "Fila de relatórios não foi informada");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Task de rede/MQTT iniciada no core %d", xPortGetCoreID());

    esp_err_t ret = mqtt_bridge_init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar NVS: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    wifi_manager_init(WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH_MODE);

    TickType_t wait_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS);
    while (!wifi_manager_is_connected()) {
        if (xTaskGetTickCount() > wait_deadline) {
            ESP_LOGW(TAG, "WiFi ainda não conectou; mantendo a task em espera");
            wait_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ret = mqtt_bridge_start_client();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Não foi possível iniciar o cliente MQTT: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (!wifi_manager_is_connected()) {
            ESP_LOGW(TAG, "WiFi indisponível, aguardando reconexão");
            vTaskDelay(pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS));
            continue;
        }

// 1. IMPORTANTE: Aumente o tamanho do payload para suportar todo o texto das harmônicas
        char payload[512]; 

        if (xQueueReceive(report_queue, &report, pdMS_TO_TICKS(MQTT_RETRY_DELAY_MS)) == pdTRUE) {
            // 2. Montamos o JSON completo, incluindo o array 'harmonicas' com as 5 frequências
            int len = snprintf(
                payload,
                sizeof(payload),
                "{\"mean_raw\":%.2f,\"rms_raw\":%.2f,\"mean_mv\":%.1f,\"rms_mv\":%.1f,\"mains_hz\":%.1f,\"mains_mag\":%.2f,\"first_raw\":%d,\"min_raw\":%d,\"max_raw\":%d,"
                "\"harmonicas\":["
                "{\"f\":%.1f,\"a\":%.2f,\"p\":%.2f},"
                "{\"f\":%.1f,\"a\":%.2f,\"p\":%.2f},"
                "{\"f\":%.1f,\"a\":%.2f,\"p\":%.2f},"
                "{\"f\":%.1f,\"a\":%.2f,\"p\":%.2f},"
                "{\"f\":%.1f,\"a\":%.2f,\"p\":%.2f}"
                "]}",
                report.mean_raw,
                report.rms_raw,
                report.mean_voltage_mv,
                report.rms_voltage_mv,
                report.mains_freq_hz,
                report.mains_magnitude,
                report.first_raw_value,
                report.min_raw_value,
                report.max_raw_value,
                // Passando os dados das 5 harmônicas extraídas pela FFT
                report.harmonics[0].frequency, report.harmonics[0].magnitude, report.harmonics[0].phase,
                report.harmonics[1].frequency, report.harmonics[1].magnitude, report.harmonics[1].phase,
                report.harmonics[2].frequency, report.harmonics[2].magnitude, report.harmonics[2].phase,
                report.harmonics[3].frequency, report.harmonics[3].magnitude, report.harmonics[3].phase,
                report.harmonics[4].frequency, report.harmonics[4].magnitude, report.harmonics[4].phase
            );

            if (len > 0 && len < (int)sizeof(payload)) {
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
                if (msg_id >= 0) {
                    ESP_LOGI(TAG, "Publicação MQTT enviada: msg_id=%d | %s", msg_id, payload);
                }
            } else {
                ESP_LOGE(TAG, "Erro: Buffer de payload muito pequeno para o JSON!");
            }
        }
    }
}
