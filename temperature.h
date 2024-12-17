/*
 * temperature.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_TEMPERATURE_H_
#define INC_TEMPERATURE_H_

#include "main.h"
#include "adc.h"

#define NTC_SENSOR_COUNT 6

typedef struct
{
  float filtered_value[ADC_CH_COUNT-2];
  float temperature_celcious[ADC_CH_COUNT-2];
}temperature_t;

//extern temperature_t temp_c;

void temperature_getTemperatureCelcious(temperature_t *temp, uint16_t *adc);
void temperature_getTemperatureSteinhart (temperature_t *temp, uint16_t *adc);
#endif /* INC_TEMPERATURE_H_ */
