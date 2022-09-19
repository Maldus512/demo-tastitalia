#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "peripherals/leds.h"
#include "peripherals/buttons.h"
#include "peripherals/system.h"
#include "peripherals/storage.h"
#include "peripherals/gatt_server.h"
#include "peripherals/battery.h"
#include "timer/timecheck.h"
#include "utils/utils.h"


static const char *TAG = "Main";


void app_main(void) {
    unsigned long timestamp  = 0;
    unsigned long sleep_ts   = 0;
    uint8_t       buttons[4] = {0};

    system_i2c_init();
    storage_init();
    leds_init();
    buttons_init();
    battery_init();
    gatt_server_init();
    gatt_server_start();

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        keypad_update_t update = {0};
        if (buttons_event(&update)) {
            ESP_LOGD(TAG, "event %i %i", update.event, update.code);

            sleep_ts = get_millis();     // Reset sleep timer

            switch (update.event) {
                case KEY_PRESS:
                    ESP_LOGI(TAG, "Button %i pressed", update.code);
                    buttons[update.code] = 1;
                    break;

                case KEY_RELEASE:
                    ESP_LOGI(TAG, "Button %i released", update.code);
                    buttons[update.code] = 0;
                    break;

                default:
                    break;
            }
            gatt_server_set_buttons(buttons);
        }

        if (is_expired(timestamp, get_millis(), 1000UL)) {
            gatt_server_set_battery(battery_get_adc_value());
            timestamp = get_millis();
        }

        if (is_expired(sleep_ts, get_millis(), 5UL * 60UL * 1000UL)) {
            ESP_LOGI(TAG, "Entering light sleep");
            gatt_server_stop();

            do {
                // Wake up every 2 seconds (2000000 microseconds)
                esp_sleep_enable_timer_wakeup(2UL * 1000UL * 1000UL);
                esp_light_sleep_start();
            } while (buttons_get_status() == 0);     // Wake up when there are no buttons pressed

            ESP_LOGI(TAG, "Waking up...");
            gatt_server_start();

            sleep_ts = get_millis();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
