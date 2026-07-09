#include "current_monitor.h"
#include "mqtt_bridge.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define MONITOR_QUEUE_LENGTH 1

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando tasks: core 0 para dados e core 1 para WiFi/MQTT");

    QueueHandle_t report_queue = xQueueCreate(MONITOR_QUEUE_LENGTH, sizeof(current_monitor_report_t));
    if (report_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar a fila de relatórios");
        vTaskDelete(NULL);
        return;
    }

    esp_err_t ret = current_monitor_init_adc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar ADC: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    BaseType_t monitor_ok = xTaskCreatePinnedToCore(
        current_monitor_task,
        "Monitor_Corrente",
        6144,
        report_queue,
        6,
        NULL,
        0);

    if (monitor_ok != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar task de dados da corrente");
        vTaskDelete(NULL);
        return;
    }

    BaseType_t mqtt_ok = xTaskCreatePinnedToCore(
        mqtt_bridge_task,
        "MQTT_Bridge",
        6144,
        report_queue,
        5,
        NULL,
        1);

    if (mqtt_ok != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar task de rede/MQTT");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}