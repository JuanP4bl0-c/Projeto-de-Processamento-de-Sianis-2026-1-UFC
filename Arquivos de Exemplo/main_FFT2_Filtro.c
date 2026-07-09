// Modificado para aplicar um filtro IIR passa-baixas com fc = 100 Hz antes da segunda FFT
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "esp_log.h"
#include "esp_dsp.h"

#define TAG "ADC_FFT"

#define N_SAMPLES 1024
adc_channel_t ADC_CHANNEL[1] = {ADC_CHANNEL_5};
#define SAMPLE_FREQ_HZ 20000

static adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t cb_task_handle = NULL;

static float adc_buffer[N_SAMPLES];
static float filtered_buffer[N_SAMPLES];
static int adc_index = 0;

static volatile bool buffer_full = false;

// FFT buffers
static float window[N_SAMPLES] __attribute__((aligned(16)));
static float fft_input[N_SAMPLES * 2] __attribute__((aligned(16))); // complex array: real+imag
static float mag_db[N_SAMPLES / 2];

// Filtro IIR passa-baixas
static float coeffs_lp[5];
static float w_lp[5] = {0};

static bool IRAM_ATTR adc_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR(cb_task_handle, &must_yield);
    return (must_yield == pdTRUE);
}

static void cbTask(void *param) {
    uint8_t buf[128];
    uint32_t ret_num = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        esp_err_t ret = adc_continuous_read(adc_handle, buf, sizeof(buf), &ret_num, 0);
        if (ret == ESP_OK && ret_num > 0) {
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *data = (adc_digi_output_data_t *)&buf[i];
                if (data->type2.channel == ADC_CHANNEL[0]) {
                    float voltage = (float)data->type2.data * 3.3f / 4095.0f;
                    adc_buffer[adc_index++] = voltage;
                    if (adc_index >= N_SAMPLES) {
                        adc_index = 0;
                        buffer_full = true;
                    }
                }
            }
        }
    }
}

static void fft_process_and_print(float *input, const char *label) {
    for (int i = 0; i < N_SAMPLES; i++) {
        fft_input[2 * i] = input[i] * window[i];
        fft_input[2 * i + 1] = 0.0f;
    }
    dsps_fft2r_fc32(fft_input, N_SAMPLES);
    dsps_bit_rev_fc32(fft_input, N_SAMPLES);
    dsps_cplx2reC_fc32(fft_input, N_SAMPLES);
    for (int i = 0; i < N_SAMPLES / 2; i++) {
        float real = fft_input[2 * i];
        float imag = fft_input[2 * i + 1];
        float mag = real * real + imag * imag;
        mag_db[i] = 10.0f * log10f(mag / N_SAMPLES + 1e-12f);
    }
    printf("---%s_START---\n", label);
    for (int i = 0; i < N_SAMPLES / 2; i++) {
        float freq = (float)i * SAMPLE_FREQ_HZ / N_SAMPLES;
        printf("%.1f,%.6f\n", freq, mag_db[i]);
    }
    printf("---%s_END---\n", label);
}

static void fftTask(void *param) {
    dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    dsps_wind_hann_f32(window, N_SAMPLES);
    dsps_biquad_gen_lpf_f32(coeffs_lp, 100.0f / SAMPLE_FREQ_HZ, 0.707f); // fc=100Hz, Q=0.707
    while (1) {
        if (buffer_full) {
            buffer_full = false;
            memcpy(filtered_buffer, adc_buffer, sizeof(adc_buffer));
            dsps_biquad_f32(filtered_buffer, filtered_buffer, N_SAMPLES, coeffs_lp, w_lp);
            fft_process_and_print(adc_buffer, "FFT_RAW");
            fft_process_and_print(filtered_buffer, "FFT_FILTRADO");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .conv_frame_size = 12,
        .max_store_buf_size = 24,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));
    adc_continuous_config_t adc_config = {
        .pattern_num = 1,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .sample_freq_hz = SAMPLE_FREQ_HZ,
    };
    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL[0],
        .bit_width = ADC_BITWIDTH_12,
        .unit = ADC_UNIT_1,
    };
    adc_config.adc_pattern = &pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_callback,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    xTaskCreate(cbTask, "ADC Callback Task", 4096, NULL, 5, &cb_task_handle);
    xTaskCreate(fftTask, "FFT Task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "Setup completo.");
}
