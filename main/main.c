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


static const char *TAG = "Main";


void app_main(void) {
    uint8_t buttons[4] = {0};
    model_t model;

    system_i2c_init();
    storage_init();
    leds_init();
    buttons_init();
    battery_init();
    gatt_server_init();

    model_init(&model);
    // view_init(&model);
    controller_init(&model);

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        keypad_update_t update = {0};
        if (buttons_event(&update)) {
            ESP_LOGI(TAG, "event %i %i", update.event, update.code);
            switch (update.event) {
                case KEY_PRESS:
                    buttons[update.code] = 1;
                    break;

                case KEY_LONGCLICK:
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

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
