/**
 * @file battery.c
 * @author Maldus512 ()
 * @brief Battery management module
 * @version 0.1
 * @date 2022-09-05
 *
 * @copyright Copyright (c) 2022
 *
 */

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


#define DEFAULT_VREF 1100     // Use adc2_vref_to_gpio() to obtain a better estimate
#define SAMPLES_NUM  10       // Multisampling


static void check_efuse(void);
static void adc_timer(TimerHandle_t timer);


static const char            *TAG     = "Battery";
static const adc1_channel_t   channel = HAP_BATTERY_CHANNEL;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width   = ADC_WIDTH_BIT_12;
static const adc_atten_t      atten   = ADC_ATTEN_DB_0;
static const adc_unit_t       unit    = ADC_UNIT_1;

static SemaphoreHandle_t             sem       = NULL;
static TimerHandle_t                 timer     = NULL;
static esp_adc_cal_characteristics_t adc_chars = {0};

static uint8_t  full_circle             = 0;
static size_t   adc_index               = 0;
static uint32_t adc_values[SAMPLES_NUM] = {0};


void battery_init(void) {
    sem = xSemaphoreCreateMutex();

    timer = xTimerCreate(TAG, 200, 1, NULL, adc_timer);

    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Configure ADC
    ESP_ERROR_CHECK(adc1_config_width(width));
    ESP_ERROR_CHECK(adc1_config_channel_atten(channel, atten));

    // Characterize ADC
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars);
}


uint32_t battery_get_adc_value(void) {
    xSemaphoreTake(sem, portMAX_DELAY);
    uint64_t total = 0;
    size_t   i     = 0;
    size_t   end   = full_circle ? SAMPLES_NUM : adc_index;

    if (end == 0) {
        xSemaphoreGive(sem);
        return 0;
    }

    for (i = 0; i < end; i++) {
        total += adc_values[i];
    }
    xSemaphoreGive(sem);

    return total / i;
}


/**
 * @brief Check whether calibration values are saved in the efuse
 *
 */
static void check_efuse(void) {
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGW(TAG, "Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
}


/**
 * @brief ADC read recurring timer
 *
 * @param timer
 */
static void adc_timer(TimerHandle_t timer) {
    (void)timer;

    xSemaphoreTake(sem, portMAX_DELAY);
    // Multisampling
    adc_values[adc_index] = adc1_get_raw((adc1_channel_t)channel);

    if (adc_index + 1 == SAMPLES_NUM) {
        full_circle = 1;
    }

    adc_index = (adc_index + 1) % SAMPLES_NUM;
    xSemaphoreGive(sem);

    // Convert adc_reading to voltage in mV
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    vTaskDelay(pdMS_TO_TICKS(1000));
}