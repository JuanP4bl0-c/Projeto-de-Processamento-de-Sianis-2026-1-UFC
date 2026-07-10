#include <stdio.h>
#include <string.h>

#include "mqtt_bridge.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

static const char *TAG = "MQTT_BRIDGE";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;

// ---------------------- WIFI ----------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, tentando reconectar...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi conectado, IP obtido.");
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_MODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------------------- MQTT ----------------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado ao broker.");
            s_mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT desconectado.");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Erro MQTT.");
            break;
        default:
            break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

// ---------------------- API PÚBLICA ----------------------
void mqtt_bridge_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    mqtt_app_start();
}

bool mqtt_bridge_is_connected(void)
{
    return s_mqtt_connected;
}

void mqtt_bridge_publish_report(const current_monitor_report_t *report)
{
    if (report == NULL) {
        return;
    }

    if (!s_mqtt_connected || s_mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT não conectado, pulando publicação.");
        return;
    }

    // Estático para não pressionar a stack da task com um buffer de ~2KB.
    static char payload[2048];
    int offset = 0;

    offset += snprintf(payload + offset, sizeof(payload) - offset,
                        "{\"rms_mv\":%.2f,\"mains_hz\":%.2f,\"rms_raw\":%.2f,\"harmonicas\":[",
                        report->rms_voltage_mv, report->mains_freq_hz, report->rms_raw);

    bool first = true;
    for (int i = 0; i < REPORT_PEAKS && offset < (int)sizeof(payload) - 64; i++) {
        if (report->harmonics[i].frequency <= 0.0f) {
            continue;
        }
        offset += snprintf(payload + offset, sizeof(payload) - offset,
                            "%s{\"f\":%.2f,\"a\":%.2f}",
                            first ? "" : ",",
                            report->harmonics[i].frequency,
                            report->harmonics[i].magnitude);
        first = false;
    }

    offset += snprintf(payload + offset, sizeof(payload) - offset, "]}");

    esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_SIGNAL, payload, offset, 1, 0);
    ESP_LOGI(TAG, "Publicado em %s (%d bytes, %s harmonicas)",
             MQTT_TOPIC_SIGNAL, offset, first ? "0" : ">0");
}