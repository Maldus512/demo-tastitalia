#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "hardwareprofile.h"
#include "buttons.h"
#include "keypad/keypad.h"
#include "utils/utils.h"


#define TP_THRESHOLD 10


static uint16_t read_button(buttons_t button);
static void     tp_timer(TimerHandle_t timer);
void app_test(void);


static const char *TAG = "Buttons";

static const touch_pad_t pads[] = {
    HAP_TP1,
    HAP_TP2,
    HAP_TP3,
    HAP_TP4,
};

static QueueHandle_t queue             = NULL;
static uint16_t      initial_values[4] = {0};
static keypad_key_t  buttons[]         = {
    KEYPAD_KEY(0x01, BUTTONS_LEFT_ARROW),
    KEYPAD_KEY(0x02, BUTTONS_LEFT_SYMBOL),
    KEYPAD_KEY(0x04, BUTTONS_RIGHT_ARROW),
    KEYPAD_KEY(0x08, BUTTONS_RIGHT_SYMBOL),
    KEYPAD_NULL_KEY,
};


void buttons_init(void) {
    static StaticQueue_t queue_buffer;
    static uint8_t       queue_storage[sizeof(keypad_update_t) * 16];
    queue = xQueueCreateStatic(sizeof(queue_storage) / sizeof(keypad_update_t), sizeof(keypad_update_t), queue_storage,
                               &queue_buffer);

    /* Initialize touch pad peripheral. */
    ESP_ERROR_CHECK(touch_pad_init());
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    for (size_t i = 0; i < sizeof(pads) / sizeof(pads[0]); i++) {
        ESP_ERROR_CHECK(touch_pad_config(pads[i], 0));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Calibrating values");
    for (buttons_t i = 0; i < sizeof(pads) / sizeof(pads[i]); i++) {
        uint64_t average_value = 0;

        for (size_t j = 0; j < 10; j++) {
            average_value += read_button(i);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        initial_values[i] = average_value / 10;
    }

    static StaticTimer_t timer_buffer;
    TimerHandle_t        timer = xTimerCreateStatic(TAG, pdMS_TO_TICKS(50), 1, NULL, tp_timer, &timer_buffer);
    xTimerStart(timer, portMAX_DELAY);
}


uint8_t buttons_event(keypad_update_t *update) {
    return xQueueReceive(queue, update, 0);
}


static uint16_t read_button(buttons_t button) {
    uint16_t touch_value = 0;
    touch_pad_read(pads[button], &touch_value);     // read raw data.
    return touch_value;
}


static void tp_timer(TimerHandle_t timer) {
    unsigned int keymap = 0;

    uint16_t values[4] = {0};

    for (buttons_t i = 0; i < sizeof(pads) / sizeof(pads[0]); i++) {
        uint16_t touch_value = read_button(i);
        values[i]            = touch_value;
        if (touch_value < initial_values[i] - TP_THRESHOLD) {
            keymap |= (1 << i);
        }
    }

    //ESP_LOGI(TAG, "%6i %6i %6i %6i", initial_values[0], initial_values[1], initial_values[2], initial_values[3]);
    ESP_LOGI(TAG, "%6i %6i %6i %6i", values[0], values[1], values[2], values[3]);

    keypad_update_t update = keypad_routine(buttons, 100, 1500, 100, get_millis(), keymap);
    if (update.event != KEY_NOTHING) {
        xQueueSend(queue, &update, 0);
    }
}


#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (0)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
/*
  Read values sensed at all available touch pads.
 Print out values in a loop on a serial monitor.
 */
static void tp_example_read_task(void *pvParameter)
{
    uint16_t touch_value;
    uint16_t touch_filter_value;
#if TOUCH_FILTER_MODE_EN
    printf("Touch Sensor filter mode read, the output format is: \nTouchpad num:[raw data, filtered data]\n\n");
#else
    printf("Touch Sensor normal mode read, the output format is: \nTouchpad num:[raw data]\n\n");
#endif
    while (1) {
        printf("%04X = ", touch_pad_get_status());
        for (int i = 0; i < 4; i++) {
#if TOUCH_FILTER_MODE_EN
            // If open the filter mode, please use this API to get the touch pad count.
            touch_pad_read_raw_data(i, &touch_value);
            touch_pad_read_filtered(i, &touch_filter_value);
            printf("T%d:[%4d,%4d] ", i, touch_value, touch_filter_value);
#else
            touch_pad_read(pads[i], &touch_value);
            printf("T%d:[%4d] ", i, touch_value);
#endif
        }
        printf("\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

static void tp_example_touch_pad_init(void)
{
    for (int i = 0;i< 4;i++) {
        touch_pad_config(pads[i], TOUCH_THRESH_NO_USE);
    }
}

void app_test(void)
{
// Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    ESP_ERROR_CHECK(touch_pad_init());
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    tp_example_touch_pad_init();
#if TOUCH_FILTER_MODE_EN
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
#endif
    // Start task to read values sensed by pads
    xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
}