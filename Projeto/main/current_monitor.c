#include "current_monitor.h"

#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

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
    float re;
    float im;
} complex_float_t;

typedef struct {
    int bin;
    float magnitude;
    float phase; // Fase em radianos
} spectral_peak_t;

static const char *TAG = "CURRENT_MONITOR";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static float s_samples[FFT_SIZE];
static complex_float_t s_fft_buffer[FFT_SIZE];
static int s_last_first_raw_value = 0;
static int s_last_min_raw_value = 0;
static int s_last_max_raw_value = 0;

static float adc_raw_to_voltage_mv(int raw_value)
{
    return ((float)raw_value * ADC_VREF_MV) / ADC_MAX_COUNTS;
}

static float magnitude_of(const complex_float_t *value)
{
    return sqrtf((value->re * value->re) + (value->im * value->im));
}

static float phase_of(const complex_float_t *value)
{
    return atan2f(value->im, value->re);
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

static void fft_radix2(complex_float_t *buffer, size_t size)
{
    for (size_t i = 1, j = 0; i < size; ++i) {
        size_t bit = size >> 1;

        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j) {
            complex_float_t temp = buffer[i];
            buffer[i] = buffer[j];
            buffer[j] = temp;
        }
    }

    for (size_t len = 2; len <= size; len <<= 1) {
        float angle = -2.0f * (float)M_PI / (float)len;
        complex_float_t wlen = {
            .re = cosf(angle),
            .im = sinf(angle),
        };

        for (size_t i = 0; i < size; i += len) {
            complex_float_t w = { .re = 1.0f, .im = 0.0f };

            for (size_t j = 0; j < len / 2; ++j) {
                complex_float_t u = buffer[i + j];
                complex_float_t v = {
                    .re = buffer[i + j + len / 2].re * w.re - buffer[i + j + len / 2].im * w.im,
                    .im = buffer[i + j + len / 2].re * w.im + buffer[i + j + len / 2].im * w.re,
                };

                buffer[i + j].re = u.re + v.re;
                buffer[i + j].im = u.im + v.im;
                buffer[i + j + len / 2].re = u.re - v.re;
                buffer[i + j + len / 2].im = u.im - v.im;

                complex_float_t next_w = {
                    .re = w.re * wlen.re - w.im * wlen.im,
                    .im = w.re * wlen.im + w.im * wlen.re,
                };
                w = next_w;
            }
        }
    }
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
    if (s_adc_handle != NULL) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = CURRENT_SENSOR_ADC_UNIT,
    };

    esp_err_t ret = adc_oneshot_new_unit(&unit_config, &s_adc_handle);
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

    ESP_LOGI(TAG, "ADC configurado no GPIO %d (ADC1_CH%d)", CURRENT_SENSOR_GPIO, CURRENT_SENSOR_ADC_CHANNEL);
    return ESP_OK;
}

static void current_monitor_acquire_window(float *samples, size_t sample_count)
{
    const int64_t period_us = SAMPLE_PERIOD_US;
    int64_t next_sample_us = esp_timer_get_time();
    int first_raw_value = -1;
    int min_raw_value = INT32_MAX;
    int max_raw_value = INT32_MIN;

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
        s_last_max_raw_value = max_raw_value;

        ESP_LOGI(TAG, "Entrada ADC0: bruto=%d, tensao=%.1f mV, faixa=[%d, %d]",
                 first_raw_value,
                 adc_raw_to_voltage_mv(first_raw_value),
                 min_raw_value,
                 max_raw_value);
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

    for (size_t index = 0; index < sample_count; ++index) {
        float centered = samples[index] - mean;
        energy += centered * centered;

        float window = 0.5f * (1.0f - cosf((2.0f * (float)M_PI * (float)index) / (float)(sample_count - 1)));
        s_fft_buffer[index].re = centered * window;
        s_fft_buffer[index].im = 0.0f;
    }

    float rms = sqrtf(energy / (float)sample_count);
    float mean_voltage_mv = adc_raw_to_voltage_mv((int)lroundf(mean));
    float rms_voltage_mv = adc_raw_to_voltage_mv((int)lroundf(rms));
    fft_radix2(s_fft_buffer, sample_count);

    spectral_peak_t peaks[REPORT_PEAKS] = {0};
    for (int index = 0; index < REPORT_PEAKS; ++index) {
        peaks[index].bin = 0;
        peaks[index].magnitude = 0.0f;
        peaks[index].phase = 0.0f; // NOVO
    }

    int min_bin = (20 * (int)sample_count) / SAMPLE_RATE_HZ;
    if (min_bin < 1) {
        min_bin = 1;
    }

    int max_bin = ((SAMPLE_RATE_HZ / 2) * (int)sample_count) / SAMPLE_RATE_HZ;
    if (max_bin > (int)(sample_count / 2)) {
        max_bin = (int)(sample_count / 2);
    }

    // NOVO: Busca apenas por Picos Locais Local Maxima para evitar vazamento espectral
    for (int bin = min_bin; bin < max_bin; ++bin) {
        
        float mag_center = magnitude_of(&s_fft_buffer[bin]);
        float mag_left   = magnitude_of(&s_fft_buffer[bin - 1]);
        float mag_right  = magnitude_of(&s_fft_buffer[bin + 1]);

        // Só considera uma harmônica válida se for o topo do "morro"
        if (mag_center > mag_left && mag_center > mag_right) {
            float phase = phase_of(&s_fft_buffer[bin]);
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
        float magnitude = magnitude_of(&s_fft_buffer[bin]);
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