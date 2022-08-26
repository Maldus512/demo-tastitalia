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

    HRS_IDX_NB,
};


void gatt_server_init(void);
void gatt_server_set_buttons(uint8_t buttons[4]);

#endif