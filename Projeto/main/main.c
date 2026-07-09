#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h" // Para esp_rom_delay_us
#include <math.h>

#define N_SAMPLES 1024
#define ADC_CHANNEL ADC_CHANNEL_0

static const char *TAG = "FFT_ADC";

// Buffers alinhados para esp-dsp (necessário para performance)
static __attribute__((aligned(16))) float wind[N_SAMPLES];
static __attribute__((aligned(16))) float adc_data[N_SAMPLES * 2]; // Intercalado Real/Imag

void app_main()
{
    // 1. Inicializa FFT
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(wind, N_SAMPLES);

    // 2. Configura ADC1 (Canal 0)
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config);

    ESP_LOGI(TAG, "Capturando e processando FFT a 4kHz...");

    while (1) {
        // 3. Captura amostras
        for (int i = 0; i < N_SAMPLES; i++) {
            int raw;
            adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
            
            // Centraliza o sinal (Ajuste o 1340 conforme seu offset medido)
            adc_data[i * 2 + 0] = (float)(raw - 2200) / 1800.0f;
            adc_data[i * 2 + 1] = 0; 
            
            // Aplica janela de Hann
            adc_data[i * 2 + 0] *= wind[i]; 

            int min_val = 4095;
            int max_val = 0;
            for (int i = 0; i < N_SAMPLES; i++) {
                // ... (seu código de leitura)
                if (raw < min_val) min_val = raw;
                if (raw > max_val) max_val = raw;
            }
            ESP_LOGI(TAG, "ADC Range: [%d, %d]", min_val, max_val);
            
            // Delay de 250us -> 4kHz de sampling rate
            esp_rom_delay_us(250); 



        }

        // 4. Executa FFT
        dsps_fft2r_fc32(adc_data, N_SAMPLES);
        dsps_bit_rev_fc32(adc_data, N_SAMPLES);
        dsps_cplx2reC_fc32(adc_data, N_SAMPLES);

        // 5. Calcula magnitude em dB e prepara para visualização
        for (int i = 0; i < N_SAMPLES / 2; i++) {
            float re = adc_data[i * 2 + 0];
            float im = adc_data[i * 2 + 1];
            float mag = sqrtf(re * re + im * im);
            adc_data[i] = 10 * log10f((mag + 0.000001f) / N_SAMPLES);
        }

        // 6. Imprime espectro no terminal
        ESP_LOGW(TAG, "Espectro (dB):");
        dsps_view(adc_data, N_SAMPLES / 2, 64, 10, -60, 40, '|');

        

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}