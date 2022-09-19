#ifndef GATT_SERVER_H_INCLUDED
#define GATT_SERVER_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum {
    IDX_SVC,
    IDX_CHAR_BUTTONS,
    IDX_CHAR_VAL_BUTTONS,

    IDX_CHAR_LEDS,
    IDX_CHAR_VAL_LEDS,

    IDX_CHAR_BATTERY,
    IDX_CHAR_VAL_BATTERY,

    HRS_IDX_NB,
};


/**
 * @brief GATT server initialization
 * 
 */
void gatt_server_init(void);


void gatt_server_start(void);


void gatt_server_stop(void);


/**
 * @brief Change the buttons state to be read from the GATT server
 * 
 * @param buttons 
 */
void gatt_server_set_buttons(uint8_t buttons[4]);


/**
 * @brief Change the battery level state to be read from the GATT server
 * 
 * @param battery_adc_level 
 */
void gatt_server_set_battery(uint32_t battery_adc_level);

#endif