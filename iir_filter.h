/*
 * iir_filter.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_IIR_FILTER_H_
#define INC_IIR_FILTER_H_

#include "main.h"
#include "adc.h"

#define IIR_TEMP_ALPHA 0.5
#define IIR_CURRENT_ALPHA 0.5
#define IIR_ENCODER_PULSE_ALPHA 0.1

typedef struct
{
  float alpha;
  float alpha1;
  float alpha2;
  float out[ADC_CH_COUNT-2];
  float current;
  float encoderPulse;
}iir_filter_t;

extern iir_filter_t filter;

void iir_filter_init(iir_filter_t *filt);
float iir_filter_temperature(iir_filter_t *filt, uint16_t value, uint8_t iter);
float iir_filter_current(iir_filter_t *filt, uint16_t value);
float iir_filter_encoderPulse(iir_filter_t *filt, uint32_t value);

#endif /* INC_IIR_FILTER_H_ */
