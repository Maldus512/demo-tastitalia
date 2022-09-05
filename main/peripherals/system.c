/**
 * @file system.c
 * @author Maldus512 ()
 * @brief Common system functions
 * @version 0.1
 * @date 2022-09-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <driver/i2c.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "hardwareprofile.h"
#include "system.h"


static const char *TAG = "System";


void system_i2c_init(void) {
    ESP_LOGI(TAG, "Initializing I2C master port");

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = HAP_SDA,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = HAP_SCL,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    ESP_LOGI(TAG, "Setting I2C master configuration...");
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_LOGI(TAG, "Installing I2C master driver...");
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0,
                                       0 /*I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE */,
                                       0 /* intr_alloc_flags */));
    //ESP_ERROR_CHECK(i2c_set_timeout(I2C_NUM_0, 0xFFFFF));
}