#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "model/model.h"
#include "controller/controller.h"
#include "peripherals/leds.h"
#include "peripherals/buttons.h"
#include "peripherals/system.h"


static const char *TAG = "Main";


void app_main(void) {
    model_t model;

    system_i2c_init();
    leds_init();
    buttons_init();

    model_init(&model);
    // view_init(&model);
    controller_init(&model);

    ESP_LOGI(TAG, "Begin main loop");
    for (;;) {
        keypad_update_t update = {0};
        if (buttons_event(&update)) {
            ESP_LOGD(TAG, "event %i %i", update.event, update.code);
            switch (update.event) {
                case KEY_PRESS:
                    leds_fade(update.code, 60, 200);
                    break;

                case KEY_LONGCLICK:
                    leds_fade(update.code, 100, 500);
                    break;

                case KEY_RELEASE:
                    leds_set(update.code, 0);
                    break;

                default:
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
