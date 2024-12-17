/*
 * iir_filter.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#include "main.h"
#include "iir_filter.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "math.h"
#include "stdbool.h"

iir_filter_t filter;

void iir_filter_init(iir_filter_t *filt)
{
  //Temperature filter
  if (IIR_TEMP_ALPHA > 1.0f)
  {
    filt->alpha = 1.0f;
  }
  else if (IIR_TEMP_ALPHA < 0.0f)
  {
    filt->alpha = 0.0f;
  }
  else
  {
    filt->alpha = IIR_TEMP_ALPHA;
  }

  //Current filter
  if (IIR_CURRENT_ALPHA > 1.0f)
  {
    filt->alpha1 = 1.0f;
  }
  else if (IIR_CURRENT_ALPHA < 0.0f)
  {
    filt->alpha1 = 0.0f;
  }
  else
  {
    filt->alpha1 = IIR_CURRENT_ALPHA;
  }

  //EncoderPulse filter
  if (IIR_ENCODER_PULSE_ALPHA > 1.0f)
  {
    filt->alpha2 = 1.0f;
  }
  else if (IIR_ENCODER_PULSE_ALPHA < 0.0f)
  {
    filt->alpha2 = 0.0f;
  }
  else
  {
    filt->alpha2 = IIR_ENCODER_PULSE_ALPHA;
  }

  /*Reset filter output*/
  for (uint8_t i = 0; i<= (hadc1.Init.NbrOfConversion-2); i++)
  {
    filt->out[i] = 0.0f;
  }
  filt->current = 0.0f;
  filt->encoderPulse = 0.0f;
}

float iir_filter_temperature(iir_filter_t *filt, uint16_t value, uint8_t iter)
{
  filt->out[iter] = (1 - filt->alpha)*value + filt->alpha*filt->out[iter];
  return filt->out[iter];
}

float iir_filter_current(iir_filter_t *filt, uint16_t value)
{
  filt->current = (1 - filt->alpha1)*value + filt->alpha1*filt->current;
  return filt->current;
}

float iir_filter_encoderPulse(iir_filter_t *filt, uint32_t value)
{
  filt->encoderPulse = (1 - filt->alpha2)*(float)value + filt->alpha2*filt->encoderPulse;
  return filt->encoderPulse;
}
