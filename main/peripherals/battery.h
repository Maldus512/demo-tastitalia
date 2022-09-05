#ifndef BATTERY_H_INCLUDED
#define BATTERY_H_INCLUDED


#include <stdint.h>


/**
 * @brief Initialize the battery level monitor
 * 
 */
void     battery_init(void);


/**
 * @brief Get the current battery level as ADC value
 * 
 * @return uint32_t 
 */
uint32_t battery_get_adc_value(void);


#endif