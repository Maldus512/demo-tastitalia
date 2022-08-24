#ifndef LEDS_H_INCLUDED
#define LEDS_H_INCLUDED


#include <stdint.h>


typedef enum {
    LEDS_LED_LEFT_SYMBOL = 0,
    LEDS_LED_LEFT_ARROW,
    LEDS_LED_RIGHT_ARROW,
    LEDS_LED_RIGHT_SYMBOL,
} leds_t;


void leds_init(void);
void leds_fade(leds_t led, uint8_t value, uint32_t fade_time);
void leds_set(leds_t led, uint8_t value);


#endif