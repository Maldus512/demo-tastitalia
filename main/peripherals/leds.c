#include <driver/gpio.h>
#include <driver/ledc.h>
#include "hardwareprofile.h"
#include "leds.h"


static const ledc_channel_t leds2channel[] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};


void leds_init(void) {
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,        // resolution of PWM duty
        .freq_hz         = 1000,                    // frequency of PWM signal
        .speed_mode      = LEDC_LOW_SPEED_MODE,     // timer mode
        .timer_num       = LEDC_TIMER_0,            // timer index
        .clk_cfg         = LEDC_AUTO_CLK,           // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t channel_configs[] = {
        {
            .channel             = LEDC_CHANNEL_0,
            .duty                = 0,
            .gpio_num            = HAP_LED1,
            .speed_mode          = LEDC_LOW_SPEED_MODE,
            .hpoint              = 0,
            .timer_sel           = LEDC_TIMER_0,
            .flags.output_invert = 0,
        },
        {
            .channel             = LEDC_CHANNEL_1,
            .duty                = 0,
            .gpio_num            = HAP_LED2,
            .speed_mode          = LEDC_LOW_SPEED_MODE,
            .hpoint              = 0,
            .timer_sel           = LEDC_TIMER_0,
            .flags.output_invert = 0,
        },
        {
            .channel             = LEDC_CHANNEL_2,
            .duty                = 0,
            .gpio_num            = HAP_LED3,
            .speed_mode          = LEDC_LOW_SPEED_MODE,
            .hpoint              = 0,
            .timer_sel           = LEDC_TIMER_0,
            .flags.output_invert = 1,
        },
        {
            .channel             = LEDC_CHANNEL_3,
            .duty                = 0,
            .gpio_num            = HAP_LED4,
            .speed_mode          = LEDC_LOW_SPEED_MODE,
            .hpoint              = 0,
            .timer_sel           = LEDC_TIMER_0,
            .flags.output_invert = 1,
        },
    };

    for (size_t i = 0; i < sizeof(channel_configs) / sizeof(channel_configs[0]); i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channel_configs[i]));
    }

    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}


void leds_fade(leds_t led, uint8_t value, uint32_t fade_time) {
    value        = value > 100 ? 100 : value;
    uint8_t duty = (uint8_t)((value * 0xFF) / 100);
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, leds2channel[led], duty, fade_time, LEDC_FADE_NO_WAIT);
}


void leds_set(leds_t led, uint8_t value) {
    value         = value > 100 ? 100 : value;
    uint32_t duty = (uint32_t)((((uint32_t)value) * 0xFF) / 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leds2channel[led], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leds2channel[led]);
}