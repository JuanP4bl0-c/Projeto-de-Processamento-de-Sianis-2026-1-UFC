#include "current_monitor.h"

#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

// Inclusão da biblioteca esp-dsp
#include "esp_dsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define CURRENT_SENSOR_GPIO 36
#define CURRENT_SENSOR_ADC_UNIT ADC_UNIT_1
#define CURRENT_SENSOR_ADC_CHANNEL ADC_CHANNEL_0
#define CURRENT_SENSOR_ATTENUATION ADC_ATTEN_DB_12

#define FFT_SIZE 1024
#define SAMPLE_RATE_HZ 4000
#define SAMPLE_PERIOD_US (1000000LL / SAMPLE_RATE_HZ)
#define SAMPLE_PREVIEW_COUNT 8
#define ADC_MAX_COUNTS 4095.0f
#define ADC_VREF_MV 3300.0f

typedef struct {
    int bin;
    float magnitude;
    float phase; // Fase em radianos
} spectral_peak_t;

static const char *TAG = "CURRENT_MONITOR";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static float s_samples[FFT_SIZE];

// Buffers exigidos pelo esp-dsp
// O buffer FFT precisa de espaço para Real e Imaginário intercalados (2 * FFT_SIZE)
// Alinhamento de 16 bytes melhora performance em arquiteturas que suportam instruções SIMD
static float s_fft_buffer[FFT_SIZE * 2] __attribute__((aligned(16)));
static float s_hann_window[FFT_SIZE] __attribute__((aligned(16)));
static bool s_dsp_initialized = false;

static int s_last_first_raw_value = 0;
static int s_last_min_raw_value = 0;
static int s_last_max_raw_value = 0;

static float adc_raw_to_voltage_mv(int raw_value)
{
    return ((float)raw_value * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static void current_monitor_log_sample_preview(const float *samples, size_t sample_count)
{
    size_t preview_count = sample_count < SAMPLE_PREVIEW_COUNT ? sample_count : SAMPLE_PREVIEW_COUNT;
    char preview_line[256];
    size_t offset = 0;

    offset += (size_t)snprintf(preview_line + offset, sizeof(preview_line) - offset, "ADC0 preview:");

    for (size_t index = 0; index < preview_count && offset < sizeof(preview_line); ++index) {
        int raw_value = (int)lroundf(samples[index]);
        float voltage_mv = adc_raw_to_voltage_mv(raw_value);
        offset += (size_t)snprintf(preview_line + offset,
                                   sizeof(preview_line) - offset,
                                   " [%u]=%d (%.1f mV)",
                                   (unsigned int)index,
                                   raw_value,
                                   voltage_mv);
    }

    ESP_LOGI(TAG, "%s", preview_line);
}

static void insert_peak(spectral_peak_t peaks[REPORT_PEAKS], int bin, float magnitude, float phase)
{
    if (magnitude <= peaks[REPORT_PEAKS - 1].magnitude) {
        return;
    }

    peaks[REPORT_PEAKS - 1].bin = bin;
    peaks[REPORT_PEAKS - 1].magnitude = magnitude;
    peaks[REPORT_PEAKS - 1].phase = phase;

    for (int index = REPORT_PEAKS - 1; index > 0; --index) {
        if (peaks[index].magnitude <= peaks[index - 1].magnitude) {
            break;
        }

        spectral_peak_t temp = peaks[index - 1];
        peaks[index - 1] = peaks[index];
        peaks[index] = temp;
    }
}

esp_err_t current_monitor_init_adc(void)
{
    esp_err_t ret;

    // Inicializa as tabelas internas da esp-dsp para FFT Radix-2 de até 1024 pontos
    if (!s_dsp_initialized) {
        ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao inicializar tabelas FFT da esp-dsp: %s", esp_err_to_name(ret));
            return ret;
        }
        // Gera os coeficientes da Janela de Hann via esp-dsp
        dsps_wind_hann_f32(s_hann_window, FFT_SIZE);
        s_dsp_initialized = true;
    }

    if (s_adc_handle != NULL) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = CURRENT_SENSOR_ADC_UNIT,
    };

    ret = adc_oneshot_new_unit(&unit_config, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = CURRENT_SENSOR_ATTENUATION,
    };

    ret = adc_oneshot_config_channel(s_adc_handle, CURRENT_SENSOR_ADC_CHANNEL, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar ADC no GPIO %d: %s", CURRENT_SENSOR_GPIO, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC e DSP configurados com sucesso.");
    return ESP_OK;
}

static void current_monitor_acquire_window(float *samples, size_t sample_count)
{
    const int64_t period_us = SAMPLE_PERIOD_US;
    int64_t next_sample_us = esp_timer_get_time();
    int first_raw_value = -1;
    int min_raw_value = INT32_MAX;
    int64_t max_raw_value = INT32_MIN;

    for (size_t index = 0; index < sample_count; ++index) {
        int64_t now_us = esp_timer_get_time();
        while (now_us < next_sample_us) {
            taskYIELD();
            now_us = esp_timer_get_time();
        }

        int raw_value = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_handle, CURRENT_SENSOR_ADC_CHANNEL, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Falha na leitura do ADC: %s", esp_err_to_name(ret));
            samples[index] = 0.0f;
        } else {
            samples[index] = (float)raw_value;
            if (first_raw_value < 0) {
                first_raw_value = raw_value;
            }
            if (raw_value < min_raw_value) {
                min_raw_value = raw_value;
            }
            if (raw_value > max_raw_value) {
                max_raw_value = raw_value;
            }
        }

        next_sample_us += period_us;
        if (now_us > next_sample_us + period_us) {
            next_sample_us = now_us + period_us;
        }
    }

    if (first_raw_value >= 0) {
        s_last_first_raw_value = first_raw_value;
        s_last_min_raw_value = min_raw_value;
        s_last_max_raw_value = (int)max_raw_value;

        ESP_LOGI(TAG, "Entrada ADC0: bruto=%d, tensao=%.1f mV, faixa=[%d, %d]",
                 first_raw_value,
                 adc_raw_to_voltage_mv(first_raw_value),
                 min_raw_value,
                 (int)max_raw_value);
        current_monitor_log_sample_preview(samples, sample_count);
    }
}

static void current_monitor_analyze_window(const float *samples, size_t sample_count, current_monitor_report_t *report)
{

    
    float mean = 0.0f;
    float energy = 0.0f;

    for (size_t index = 0; index < sample_count; ++index) {
        mean += samples[index];
    }
    mean /= (float)sample_count;

    // Prepara o buffer complexo intercalado aplicando o DC-Bias centralizado e a janela de Hann
    for (size_t index = 0; index < sample_count; ++index) {
        float centered = samples[index] - mean;
        energy += centered * centered;

        // Formato FC32 (Complexo Intercalado): [Real, Imag, Real, Imag...]
        s_fft_buffer[index * 2] = centered * s_hann_window[index]; // Componente Real com Janela
        s_fft_buffer[(index * 2) + 1] = 0.0f;                     // Componente Imaginário zerado
    }

    float rms = sqrtf(energy / (float)sample_count);
    float mean_voltage_mv = adc_raw_to_voltage_mv((int)lroundf(mean));
    float rms_voltage_mv = adc_raw_to_voltage_mv((int)lroundf(rms));

    // Executa a FFT Radix-2 otimizada em hardware/assembly pela esp-dsp
    dsps_fft2r_fc32(s_fft_buffer, sample_count);
    // Reordena os bins (Bit-reversal) exigido pelo algoritmo Radix-2 da biblioteca
    dsps_bit_rev_fc32(s_fft_buffer, sample_count);

    spectral_peak_t peaks[REPORT_PEAKS] = {0};

    int min_bin = (20 * (int)sample_count) / SAMPLE_RATE_HZ;
    if (min_bin < 1) {
        min_bin = 1;
    }

    int max_bin = ((SAMPLE_RATE_HZ / 2) * (int)sample_count) / SAMPLE_RATE_HZ;
    if (max_bin > (int)(sample_count / 2)) {
        max_bin = (int)(sample_count / 2);
    }

    // Busca por Picos Locais com Supressão de Vazamento Espectral
    // Janela de 3 bins garante uma distância mínima de ~11.7 Hz (3 * 3.9Hz) entre cada harmônica detectada.
    const int PEAK_SEARCH_WINDOW = 3; 
    
    for (int bin = min_bin; bin < max_bin; ++bin) {
        float re_c = s_fft_buffer[bin * 2];
        float im_c = s_fft_buffer[(bin * 2) + 1];
        float mag_center = sqrtf((re_c * re_c) + (im_c * im_c));

        bool is_local_max = true;

        // Verifica os vizinhos num raio definido para evitar capturar a "aba" do mesmo pico
        for (int w = 1; w <= PEAK_SEARCH_WINDOW; ++w) {
            
            // Vizinho à esquerda (com proteção de limite do array)
            if ((bin - w) >= 0) {
                float re_l = s_fft_buffer[(bin - w) * 2];
                float im_l = s_fft_buffer[((bin - w) * 2) + 1];
                float mag_l = sqrtf((re_l * re_l) + (im_l * im_l));
                if (mag_center <= mag_l) {
                    is_local_max = false;
                    break; // Se encontrou alguém maior, já não é o topo do pico
                }
            }
            
            // Vizinho à direita (com proteção de limite do array)
            if ((bin + w) < (sample_count / 2)) {
                float re_r = s_fft_buffer[(bin + w) * 2];
                float im_r = s_fft_buffer[((bin + w) * 2) + 1];
                float mag_r = sqrtf((re_r * re_r) + (im_r * im_r));
                if (mag_center <= mag_r) {
                    is_local_max = false;
                    break; // Se encontrou alguém maior, já não é o topo do pico
                }
            }
        }

        // Só insere no relatório se for maior que todos na janela
        if (is_local_max) {
            float phase = atan2f(im_c, re_c);
            insert_peak(peaks, bin, mag_center, phase);
        }
    }

    ESP_LOGI(TAG, "Janela ADC0: mean=%.2f raw, tensao_media=%.1f mV, rms=%.2f raw, rms_tensao=%.1f mV, taxa=%d Hz, amostras=%u",
             mean,
             mean_voltage_mv,
             rms,
             rms_voltage_mv,
             SAMPLE_RATE_HZ,
             (unsigned int)sample_count);

    int mains_start_bin = (45 * (int)sample_count) / SAMPLE_RATE_HZ;
    int mains_end_bin = (70 * (int)sample_count) / SAMPLE_RATE_HZ;
    if (mains_start_bin < 1) {
        mains_start_bin = 1;
    }
    if (mains_end_bin > (int)(sample_count / 2 - 1)) {
        mains_end_bin = (int)(sample_count / 2 - 1);
    }

    float mains_mag = 0.0f;
    float mains_freq = 0.0f;
    for (int bin = mains_start_bin; bin <= mains_end_bin; ++bin) {
        float re = s_fft_buffer[bin * 2];
        float im = s_fft_buffer[(bin * 2) + 1];
        float magnitude = sqrtf((re * re) + (im * im));
        if (magnitude > mains_mag) {
            mains_mag = magnitude;
            mains_freq = ((float)bin * (float)SAMPLE_RATE_HZ) / (float)sample_count;
        }
    }

    if (mains_mag > 0.0f) {
        ESP_LOGI(TAG, "Componente principal da rede: %.1f Hz (magnitude %.2f)", mains_freq, mains_mag);
    }

    ESP_LOGI(TAG, "Frequencias encontradas no espectro:");
    for (int index = 0; index < REPORT_PEAKS; ++index) {
        if (peaks[index].bin <= 0) {
            continue;
        }

        float frequency = ((float)peaks[index].bin * (float)SAMPLE_RATE_HZ) / (float)sample_count;
        ESP_LOGI(TAG, "  #%d -> %.1f Hz | mag %.2f | fase %.2f rad", 
                 index + 1, frequency, peaks[index].magnitude, peaks[index].phase);
    }

    if (report != NULL) {
        report->mean_raw = mean;
        report->rms_raw = rms;
        report->mean_voltage_mv = mean_voltage_mv;
        report->rms_voltage_mv = rms_voltage_mv;
        report->mains_freq_hz = mains_freq;
        report->mains_magnitude = mains_mag;
        report->first_raw_value = s_last_first_raw_value;
        report->min_raw_value = s_last_min_raw_value;
        report->max_raw_value = s_last_max_raw_value;

        for (int i = 0; i < REPORT_PEAKS; i++) {
            if (peaks[i].bin > 0) {
                report->harmonics[i].frequency = ((float)peaks[i].bin * (float)SAMPLE_RATE_HZ) / (float)sample_count;
                report->harmonics[i].magnitude = peaks[i].magnitude;
                report->harmonics[i].phase = peaks[i].phase;
            } else {
                report->harmonics[i].frequency = 0.0f;
                report->harmonics[i].magnitude = 0.0f;
                report->harmonics[i].phase = 0.0f;
            }
        }
    }
}

esp_err_t current_monitor_capture_report(current_monitor_report_t *report)
{
    if (report == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    current_monitor_acquire_window(s_samples, FFT_SIZE);
    current_monitor_analyze_window(s_samples, FFT_SIZE, report);
    return ESP_OK;
}

void current_monitor_task(void *pvParameters)
{
    QueueHandle_t report_queue = (QueueHandle_t)pvParameters;
    current_monitor_report_t report = { 0 };

    ESP_LOGI(TAG, "Task de corrente iniciada no core %d", xPortGetCoreID());

    while (1) {
        if (current_monitor_capture_report(&report) == ESP_OK && report_queue != NULL) {
            xQueueOverwrite(report_queue, &report);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}