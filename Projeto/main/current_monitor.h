#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define REPORT_PEAKS 5 
#define RAW_PREVIEW_SIZE 128 // Quantidade de amostras brutas enviadas para o Debug no Python

typedef struct {
    float frequency; 
    float magnitude; 
    float phase;     
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
    
    harmonic_t harmonics[REPORT_PEAKS]; 
    
    // NOVO: Array para armazenar as amostras brutas do ADC enviadas ao Python
    uint16_t raw_samples[RAW_PREVIEW_SIZE];
} current_monitor_report_t;

esp_err_t current_monitor_start(void);
esp_err_t current_monitor_init_adc(void);
esp_err_t current_monitor_capture_report(current_monitor_report_t *report);
void current_monitor_task(void *pvParameters);

#endif // CURRENT_MONITOR_H