/*
 * pid_regulator.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */


#include "pid_regulator.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "math.h"
#include "stdbool.h"
#include "motordc.h"
#include "MX25L4006EM_FLASH.h"

HAL_StatusTypeDef pid_infit_from_flash(pid_t *pid_data)
{
  uint8_t read_buffer[256];
  HAL_StatusTypeDef status = MX25L4006EM_ReadDataBytes(PID_FLASH_PAGE_ADDRESS, read_buffer, sizeof(*pid_data));
  if (status == HAL_OK)
    pid_data = (pid_t*)read_buffer;
  hMotor.pid_regulator = *pid_data;
  return status;
}

void pid_init(pid_t *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init)
{
  pid_data->pid_previousError = 0;
  pid_data->pid_totalError = 0;

  pid_data->Kp = kp_init;
  pid_data->Ki = ki_init;
  pid_data->Kd = kd_init;

  pid_data->pid_antiWindup = anti_windup_limit_init;
}



void pid_reset(pid_t *pid_data)
{
  pid_data->pid_previousError = 0;
  pid_data->pid_totalError = 0;
}

float pid_calculate(pid_t *pid_data, int refValue, float measuredValue)
{
  float error;
  float pid_P;
  float pid_I;
  float pid_D;

  error = refValue - measuredValue;           //Calculating system

  pid_data->pid_totalError += error;            //adding total error

  if(pid_data->pid_totalError >= pid_data->pid_antiWindup)
  {
    pid_data->pid_totalError = pid_data->pid_antiWindup;
  }
  else if (pid_data->pid_totalError <= -pid_data->pid_antiWindup)
  {
    pid_data->pid_totalError = -pid_data->pid_antiWindup;
  }

  pid_P = pid_data->Kp*error;                          //proportional
  pid_I = pid_data->Ki*pid_data->pid_totalError;       //integral
  pid_D = pid_data->Kd*(error - pid_data->pid_previousError);    //derivative

  pid_data->pid_previousError = error;        //storing previous error data

  if(pid_I >= pid_data->pid_antiWindup)
  {
    pid_I = pid_data->pid_antiWindup;
  }
  else if (pid_I <= -pid_data->pid_antiWindup)
  {
    pid_I = -pid_data->pid_antiWindup;
  }


  return (pid_P + pid_I + pid_D);
}

