#include "hardwareprofile.h"
#include "battery.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"


#define DEFAULT_VREF  1100     // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 10       // Multisampling


static void check_efuse(void);


static const char            *TAG     = "Battery";
static const adc1_channel_t   channel = HAP_BATTERY_CHANNEL;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width   = ADC_WIDTH_BIT_12;
static const adc_atten_t      atten   = ADC_ATTEN_DB_0;
static const adc_unit_t       unit    = ADC_UNIT_1;

static SemaphoreHandle_t             sem       = NULL;
static esp_adc_cal_characteristics_t adc_chars = {0};


void battery_init(void) {
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Configure ADC
    ESP_ERROR_CHECK(adc1_config_width(width));
    ESP_ERROR_CHECK(adc1_config_channel_atten(channel, atten));

    // Characterize ADC
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);

    // Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;
        // Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void check_efuse(void) {
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGW(TAG, "Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
}


static void adc_timer(TimerHandle_t timer) {
    (void)timer;
}