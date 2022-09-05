#ifndef BUTTONS_H_INCLUDED
#define BUTTONS_H_INCLUDED


#include "keypad/keypad.h"


typedef enum {
    BUTTONS_LEFT_SYMBOL = 0,
    BUTTONS_LEFT_ARROW,
    BUTTONS_RIGHT_ARROW,
    BUTTONS_RIGHT_SYMBOL,
} buttons_t;


/**
 * @brief Buttons initialization
 * 
 */
void    buttons_init(void);


/**
 * @brief Get a button event (if any)
 * 
 * @param update 
 * @return uint8_t 
 */
uint8_t buttons_event(keypad_update_t *update);


#endif