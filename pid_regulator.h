/*
 * pid_regulator.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_PID_REGULATOR_H_
#define INC_PID_REGULATOR_H_

#include "main.h"

typedef struct
{
  float Kp;                   //PID regulator settings (proportional)
  float Ki;                   //PID regulator settings (integral)
  float Kd;                   //PID regulator settings (derivative)
  float pid_previousError;    //PID regulator settings (previous error signal for derivative)
  float pid_totalError;       //PID regulator settings (total error signal for integral)
  float pid_antiWindup;       //PID regulator settings (anti-windup limit for integral)
}__attribute__((packed)) pid_t; //this one is saved to external FLASH

HAL_StatusTypeDef pid_infit_from_flash(pid_t *pid_data);

void pid_init(pid_t *pid_data, float kp_init, float ki_init, float kd_init, int anti_windup_limit_init);

void pid_reset(pid_t *pid_data);
float pid_calculate(pid_t *pid_data, int refValue, float measuredValue);
#endif /* INC_PID_REGULATOR_H_ */
