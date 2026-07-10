#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

#include "current_monitor.h"
#include "mqtt_bridge.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando monitor de corrente...");

    // 1) NVS + WiFi (STA) + cliente MQTT
    mqtt_bridge_init();

    // 2) ADC (canal 0 / GPIO36) + inicialização das tabelas de FFT e da
    //    janela de Hann da esp-dsp (mesmo procedimento de dsps_window_main.c)
    ESP_ERROR_CHECK(current_monitor_init_adc());

    // 3) Fila com 1 posição: sempre guarda o relatório mais recente
    QueueHandle_t report_queue = xQueueCreate(1, sizeof(current_monitor_report_t));
    if (report_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar fila de relatórios");
        return;
    }

    // 4) Task dedicada de aquisição + FFT (roda a cada ~250 ms e sobrescreve
    //    o relatório na fila a cada ciclo — ver current_monitor_task)
    xTaskCreatePinnedToCore(current_monitor_task, "current_monitor", 8192,
                             report_queue, 5, NULL, 1);

    // 5) Loop principal: aguarda cada novo relatório e publica via MQTT
    //    no tópico/formato que Grafico.py espera
    current_monitor_report_t report;
    while (1) {
        if (xQueueReceive(report_queue, &report, portMAX_DELAY) == pdTRUE) {
            if (mqtt_bridge_is_connected()) {
                mqtt_bridge_publish_report(&report);
            } else {
                ESP_LOGW(TAG, "MQTT não conectado, relatório descartado.");
            }
        }
    }
}