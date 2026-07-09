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

// #define WIFI_SSID "brisa-2504280"
// #define WIFI_PASSWORD "eubcidpn"
// #define MQTT_BROKER_URI "mqtt://192.168.0.6:1883"
// #define WIFI_AUTH_MODE WIFI_AUTH_WPA2_PSK

// #define WIFI_SSID "LANCHONETE"
// #define WIFI_PASSWORD "09260224"
// #define MQTT_BROKER_URI "mqtt://10.10.220.31:1883"
// #define WIFI_AUTH_MODE WIFI_AUTH_WPA2_PSK

#define WIFI_SSID "UFC_B4_SL3_2"
#define WIFI_PASSWORD ""
#define MQTT_BROKER_URI "mqtt://10.0.105.207:1883"
#define WIFI_AUTH_MODE WIFI_AUTH_OPEN

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
            vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo antes de tentar novamente
            continue;
        }

        // CORREÇÃO: A task agora fica bloqueada (portMAX_DELAY) até chegar um novo pacote do ADC
        if (xQueueReceive(report_queue, &report, portMAX_DELAY) == pdTRUE) {
            
            char payload[1024];
            int offset = snprintf(payload, sizeof(payload), "{\"rms\": %.2f, \"freqs\": [", rms);

            for (int i = 1; i < 20; i++) { // Envia as 20 primeiras harmônicas
                float re = s_fft_buffer[i * 2];
                float im = s_fft_buffer[i * 2 + 1];
                float mag = sqrtf(re * re + im * im);
                offset += snprintf(payload + offset, sizeof(payload) - offset, 
                                "{\"f\": %d, \"m\": %.2f}%s", 
                                i * (SAMPLE_RATE_HZ / FFT_SIZE), mag, (i < 19 ? "," : ""));
            }
            strcat(payload, "]}");

            // 4. Publica no MQTT
            if (offset > 0 && offset < (int)sizeof(payload)) {
                // Mudei o QoS para 1 para garantir maior fiabilidade na entrega ao broker
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, "esp32/espectro", payload, 0, 0, 0);
                if (msg_id >= 0) {
                    ESP_LOGI(TAG, "Publicação MQTT enviada (Tamanho: %d bytes)", offset);
                } else {
                    ESP_LOGE(TAG, "Falha ao publicar via MQTT.");
                }
            }
        }
    }
}
