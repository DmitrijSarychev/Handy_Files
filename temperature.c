/*
 * temperature.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

 //TODO add all includes to main.h file
#include "temperature.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "math.h"
#include "stdbool.h"
#include "iir_filter.h"
#include "motordc.h"


temperature_t temp_c;
uint16_t  temperature_sensor[NTC_SENSOR_COUNT];

//all thermistor stuff goes here TODO
#define A 0.0012996f      //Calculated value from NTC characteristics
#define B 0.0002150f      //Calculated value from NTC characteristics
#define C 0.000000094f    //Calculated value from NTC characteristics

#define NTC_UP_R 10000.0f //Upper resistor for NTC voltage divider

#define THERMISTOR_NOMINAL_R 6800.0f //Nominal thermistor resistance @25C
#define TEMPERATURE_NOMINAL  25.0f
#define BETA                 3977.0f //Beta coefficient provided by the manufacturer

void temperature_getTemperatureCelcious(temperature_t *temp, uint16_t *adc)
{
  for(uint8_t i = 0;i < (ADC_CH_COUNT-2);i++)
  {
    temp->filtered_value[i] = iir_filter_temperature(&filter, adc[i+2], i);
    float ntc_R = ((NTC_UP_R)/((4095.0/temp->filtered_value[i]) - 1));
    float ntc_Ln = logf(ntc_R);
    temp->temperature_celcious[i] = (1.0/(A + B*ntc_Ln + C*ntc_Ln*ntc_Ln*ntc_Ln)) - 273.15/*from Kelvin*/;
    if(temp->temperature_celcious[i] > motor_safety_temp_threshold_get(i))
    {
      motor_fault_OT_protection_set(i, true);
    }
  }
}

void temperature_getTemperatureSteinhart (temperature_t *temp, uint16_t *adc)
{
  for(uint8_t i = 0;i < (ADC_CH_COUNT-2);i++)
  {
    temp->filtered_value[i] = iir_filter_temperature(&filter, adc[i+2], i);
    temp->filtered_value[i] = 4095 / temp->filtered_value[i] - 1;
    temp->filtered_value[i] = NTC_UP_R / temp->filtered_value[i];
    temp->temperature_celcious[i] = temp->filtered_value[i] / THERMISTOR_NOMINAL_R;
    temp->temperature_celcious[i] = log(temp->temperature_celcious[i]);
    temp->temperature_celcious[i] /= BETA;
    temp->temperature_celcious[i] += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
    temp->temperature_celcious[i] = 1.0 / temp->temperature_celcious[i];
    temp->temperature_celcious[i] -= 273.15;

    if(temp->temperature_celcious[i] > motor_safety_temp_threshold_get(i))
    {
      motor_fault_OT_protection_set(i, true);
    }
  }
}
