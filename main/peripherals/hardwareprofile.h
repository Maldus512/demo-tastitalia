#ifndef HARDWAREPROFILE_H_INCLUDED
#define HARDWAREPROFILE_H_INCLUDED

#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/gpio.h"

/*
 * Definizioni dei pin da utilizzare
 */


#define HAP_LED1 GPIO_NUM_18
#define HAP_LED2 GPIO_NUM_19
#define HAP_LED3 GPIO_NUM_17
#define HAP_LED4 GPIO_NUM_21

#define HAP_TP1 TOUCH_PAD_NUM6
#define HAP_TP2 TOUCH_PAD_NUM7
#define HAP_TP3 TOUCH_PAD_NUM8
#define HAP_TP4 TOUCH_PAD_NUM9

#define HAP_SDA GPIO_NUM_13
#define HAP_SCL GPIO_NUM_16

#define HAP_BATTERY_CHANNEL ADC1_CHANNEL_0     // GPIO36

#endif