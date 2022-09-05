#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "model/model.h"
#include "controller/controller.h"
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
    uint8_t       buttons[4] = {0};
    model_t       model;

    system_i2c_init();
    storage_init();
    leds_init();
    buttons_init();
    battery_init();
    gatt_server_init();

    model_init(&model);
    controller_init(&model);

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        keypad_update_t update = {0};
        if (buttons_event(&update)) {
            ESP_LOGD(TAG, "event %i %i", update.event, update.code);
            switch (update.event) {
                case KEY_PRESS:
                    buttons[update.code] = 1;
                    break;

                case KEY_RELEASE:
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

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
