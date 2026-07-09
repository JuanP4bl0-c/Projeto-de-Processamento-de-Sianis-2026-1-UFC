#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/unistd.h>
#include "driver/adc_types_legacy.h"
#include "esp_err.h"
#include "freertos/FreeRTOSConfig_arch.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

adc_channel_t channels[2] = {ADC1_CHANNEL_4, ADC1_CHANNEL_5};
int ADC_DATA[2];
float voltage[2];

adc_continuous_handle_t adc_handle;
TaskHandle_t cb_task;

bool New_ADC_data = pdFALSE;

static bool IRAM_ATTR callback (adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
	BaseType_t mustYield = pdFALSE;
	vTaskNotifyGiveFromISR(cb_task, &mustYield);
	return (mustYield == pdTRUE);
}

void cbTask (void *parameters)
{
	uint8_t buf[30];
	uint32_t rxLen = 0;
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		adc_continuous_read(adc_handle, buf, 12, &rxLen, 0);
		for (int i=0; i<rxLen; i+=SOC_ADC_DIGI_RESULT_BYTES)
		{
			adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
			uint16_t channel = p->type2.channel;
			uint16_t data = p->type2.data;
			if (channel == ADC1_CHANNEL_4) 
            {   
                ADC_DATA[0] = data;
            }
            else if (channel == ADC1_CHANNEL_5) 
            {   
                ADC_DATA[1] = data;
            }
            New_ADC_data = pdTRUE;
		}
	}
}

void ADC_Init (adc_channel_t *channels, uint8_t numChannels)
{
	// handle configuration
	adc_continuous_handle_cfg_t handle_config = {
		.conv_frame_size = 12,
		.max_store_buf_size = 24,
	};
	ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_config, &adc_handle));
	
	// ADC Configuration with Channels
	adc_continuous_config_t adc_cnfig = {
		.pattern_num = numChannels,
		.conv_mode = ADC_CONV_SINGLE_UNIT_1,
		.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
		.sample_freq_hz = 20*1000
	};
	adc_digi_pattern_config_t channel_config[numChannels];
	for (int i=0; i<numChannels; i++)
	{
		channel_config[i].channel = channels[i];
		channel_config[i].atten = ADC_ATTEN_DB_12;
		channel_config[i].bit_width = ADC_BITWIDTH_12;
		channel_config[i].unit = ADC_UNIT_1;
	}
	adc_cnfig.adc_pattern = channel_config;
	ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cnfig));
	
	// Callback Configuration
	adc_continuous_evt_cbs_t cb_config = {
		.on_conv_done = callback,
	};
	ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cb_config, NULL));
	
}

void app_main(void)
{
	xTaskCreate(cbTask, "Callback Task", 4096, NULL, 0, &cb_task);
	ADC_Init(channels, 2);
	ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
	while (1)
	{
        if(New_ADC_data)
        {
            New_ADC_data = pdFALSE;
            voltage[0] = ADC_DATA[0]*3.3/4095;
            voltage[1] = ADC_DATA[1]*3.3/4095;
		    printf("%.6f,%.6f\n", voltage[0],voltage[1]);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
	}
}