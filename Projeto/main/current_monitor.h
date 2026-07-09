#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Define a quantidade máxima de picos (harmônicas) extraídos pela FFT
#define REPORT_PEAKS 5 

// Nova estrutura para carregar a Frequência, Amplitude e Fase
typedef struct {
    float frequency; // Frequência em Hz
    float magnitude; // Amplitude
    float phase;     // Fase em radianos (Crucial para remontar o sinal perfeitamente)
} harmonic_t;

typedef struct {
    float mean_raw;
    float rms_raw;
    float mean_voltage_mv;
    float rms_voltage_mv;
    float mains_freq_hz;
    float mains_magnitude;
    int first_raw_value;
    int min_raw_value;
    int max_raw_value;
    
    // Array que armazenará os dados da FFT para enviar via MQTT
    harmonic_t harmonics[REPORT_PEAKS]; 
} current_monitor_report_t;

esp_err_t current_monitor_start(void);
esp_err_t current_monitor_init_adc(void);
esp_err_t current_monitor_capture_report(current_monitor_report_t *report);
void current_monitor_task(void *pvParameters);

#endif // CURRENT_MONITOR_H