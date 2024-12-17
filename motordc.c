/*
 * SilnikDC.c
 *
 *  Created on: Feb 22, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#include <motordc.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "math.h"
#include "stdbool.h"
#include "tim.h"
#include "iir_filter.h"
#include "pid_regulator.h"
#include "temperature.h"
#include "rtc.h"
#include "MX25L4006EM_FLASH.h"

motorParam_t hMotor;
motorProgram_t hProgram;
motorSafety_t hSafety;
motorFault_t hFault;

extern uint16_t ADC_result[];

void motor_init(motorParam_t *mot/*, TIM_HandleTypeDef *tim*/)
{
  //mot->timer = tim;
  mot->resolution = ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGET1T2 * GEAR_REDUCTION;

  mot->program = MOTOR_PROGRAM_1;
  mot->pulse_count = 0;
  mot->ref_speed = 0;

  mot->speed = 0;
  mot->pwm_duty = 0;
  mot->efficiency = 0.8f;

  PWM_LEFT_OFF;
  LOW_PWM_LEFT_OFF;
  PWM_RIGHT_OFF;
  LOW_PWM_RIGHT_OFF;

  if(pid_infit_from_flash(&(hMotor.pid_regulator)) != HAL_OK)//reading from flash FAIL, load default values
    pid_init(&(hMotor.pid_regulator), MOTOR_Kp, MOTOR_Ki, MOTOR_Kd, MOTOR_ANTI_WINDUP);
  if(motor_safety_init_from_flash(&hSafety) != HAL_OK)
    motor_safety_init(&hSafety);
  if(motor_program_init_from_flash(&hProgram) != HAL_OK)
    motor_program_init(&hProgram);
  motor_faults_reset();
}


void motor_program_init(motorProgram_t *program)
{
  program->cycle_count = 1000;
  program->cycle_count_CCW = 500;
  program->cycle_count_CW = 500;
  program->cycle_revolutions = 10;
  program->cycle_time_CCW = 0;
  program->cycle_time_CW = 0;
  program->cycle_time_idle = 0;
  program->ramp_up = MOTOR_SAFETY_RAMP_UP_MAX;
  program->ramp_down = MOTOR_SAFETY_RAMP_DOWN_MAX;
}

HAL_StatusTypeDef motor_safety_init_from_flash (motorSafety_t *safety)
{
  uint8_t read_buffer[256];
  HAL_StatusTypeDef status = MX25L4006EM_ReadDataBytes(SAFETY_FLASH_PAGE_ADDRESS, read_buffer, sizeof(*safety));
  if (status == HAL_OK)
    safety = (motorSafety_t*)read_buffer;
  hSafety = *safety;
  return status;
}

HAL_StatusTypeDef motor_program_init_from_flash (motorProgram_t *program)
{
  uint8_t read_buffer[256];
  HAL_StatusTypeDef status = MX25L4006EM_ReadDataBytes(PROGRAM_FLASH_PAGE_ADDRESS, read_buffer, sizeof(*program));
  if (status == HAL_OK)
    program = (motorProgram_t*)read_buffer;
  hProgram = *program;
  return status;
}

void motor_safety_init(motorSafety_t *safety)
{
  safety->safety_max_current = MOTOR_SAFETY_OC_MAX;
  safety->safety_max_voltage = MOTOR_SAFETY_OV_MAX;
  safety->safety_min_voltage = MOTOR_SAFETY_UV_MIN;
  safety->safety_max_ramp_up = MOTOR_SAFETY_RAMP_UP_MAX;
  safety->safety_max_rpm = MOTOR_SAFETY_RPM_MAX;
  safety->safety_max_ramp_down = MOTOR_SAFETY_RAMP_DOWN_MAX;

  for (uint8_t i = 0; i < NTC_SENSOR_COUNT; i++)
  {
    safety->safety_temp_threshold[i] = MOTOR_SAFETY_TEMP_MAX;
  }
}

void motor_reset(motorParam_t *mot)
{
  //mot->pulse_count = 0;
  mot->status = OFF;
  mot->ref_speed_ramp = 0;
  mot->speed = 0;
  mot->pwm_duty = 0;

  PWM_LEFT_OFF;
  LOW_PWM_LEFT_OFF;
  PWM_RIGHT_OFF;
  LOW_PWM_RIGHT_OFF;
  pid_reset(&(mot->pid_regulator));
}

void motor_program_set(motorParam_t *mot, uint8_t program)
{
  mot->program = program;
}
uint8_t motor_program_get(void)
{
  return hMotor.program;
}

void motor_program_run_toggle(motorParam_t *mot)
{
  mot->program_run ^=1;
}

void motor_program_run_set(uint8_t status)
{
  if(status == ON || status == OFF)
  hMotor.program_run = status;
}

uint8_t motor_program_run_get(void)
{
  return hMotor.program_run;
}

void motor_status_toggle(void)
{
  hMotor.status ^= 0x01;
}

uint8_t motor_status_get(void)
{
  return hMotor.status;
}

void motor_status_set(uint8_t status)
{
  if(status == ON || status == OFF)
    hMotor.status = status;
}

void motor_mode_set(uint8_t value)
{
  if(value == AUTO || value == MANUAL)
    hMotor.mode = value;
}

uint8_t motor_mode_get(void)
{
  return hMotor.mode;
}

void motor_refSpeed_set(motorParam_t *mot, float speed)
{
  if ((speed != mot->ref_speed) && (speed <= motor_safety_max_rpm_get()) && (speed >= -motor_safety_max_rpm_get()))
  {
    pid_reset(&(mot->pid_regulator));
    hMotor.ref_speed = speed;
  }
}

float motor_refSpeed_get(void)
{
  return hMotor.ref_speed;
}

float motor_speed_get(void)
{
  return hMotor.speed;             //get current speed value
}

void motor_speed_update(motorParam_t *mot, motorProgram_t *program)
{
  //float speed = iir_filter_encoderPulse(&filter, mot->pulse_count);
  float speed = mot->pulse_count;

  if((speed == 0) && (mot->pwm_duty > 10.0f))
    motor_fault_encoder_fail_set(true);

  if(mot->direction == CW)
  {
    mot->speed = (speed * TIMER_FREQ * MINUTE)/mot->resolution;

    /*Updating the speed using ramp value*/
    if(mot->ref_speed_ramp <= mot->ref_speed)
    {
      mot->ref_speed_ramp += (program->ramp_up/TIMER_FREQ);
      if(mot->ref_speed_ramp > mot->ref_speed)
        mot->ref_speed_ramp = mot->ref_speed;
    }
    else
    {
      mot->ref_speed_ramp -= (program->ramp_down/TIMER_FREQ);
      if(mot->ref_speed_ramp < mot->ref_speed)
        mot->ref_speed_ramp = mot->ref_speed;
    }
    /*Updating the speed using ramp value*/
  }
  else//if(mot->direction == CCW)
  {
    mot->speed = -(speed * TIMER_FREQ * MINUTE)/mot->resolution;

    /*Updating the speed using ramp value*/
    if(mot->ref_speed_ramp <= mot->ref_speed)
    {
      mot->ref_speed_ramp += (program->ramp_down/TIMER_FREQ);
      if(mot->ref_speed_ramp > mot->ref_speed)
        mot->ref_speed_ramp = mot->ref_speed;
    }
    else
    {
      mot->ref_speed_ramp -= (program->ramp_up/TIMER_FREQ);
      if(mot->ref_speed_ramp < mot->ref_speed)
        mot->ref_speed_ramp = mot->ref_speed;
    }
    /*Updating the speed using ramp value*/
  }



  float output = pid_calculate(&mot->pid_regulator, /*mot->ref_speed*/mot->ref_speed_ramp, mot->speed);

  mot->pwm_duty += output;

  if (mot->pwm_duty > PWM_DUTY_MAX)
  {
    mot->pwm_duty = PWM_DUTY_MAX;
  }
  else if (mot->pwm_duty < -PWM_DUTY_MAX)
  {
    mot->pwm_duty = -PWM_DUTY_MAX;
  }

  if (mot->pwm_duty >= 0)
  {
    PWM_LEFT_OFF;
    if(mot->direction == CCW)
      LOW_PWM_RIGHT_OFF;

    mot->direction = CW;
    motor_pwmDutyCycle_set(&htim1, mot->pwm_duty, mot->direction);

    if(mot->pwm_duty < RECT_DUTY_MIN)
    {
      LOW_PWM_RIGHT_OFF;
      LOW_PWM_LEFT_OFF;
      PWM_RIGHT_OFF;
    }
    else
    {
      PWM_RIGHT_ON;
      LOW_PWM_LEFT_ON;
      LOW_PWM_RIGHT_ON;
    }
  }
  else
  {
    PWM_RIGHT_OFF;
    if(mot->direction == CW)
      LOW_PWM_LEFT_OFF;

    mot->direction = CCW;
    motor_pwmDutyCycle_set(&htim1, -mot->pwm_duty, mot->direction);
    //LOW_PWM_RIGHT_ON;
    //PWM_LEFT_ON;
    if(-mot->pwm_duty < RECT_DUTY_MIN)
    {
      PWM_LEFT_OFF;
      LOW_PWM_LEFT_OFF;
      LOW_PWM_RIGHT_OFF;
    }
    else
    {
      LOW_PWM_RIGHT_ON;
      PWM_LEFT_ON;
      LOW_PWM_LEFT_ON;
    }
  }
}

void motor_count_store(motorParam_t *mot)
{
  mot->pulse_count = htim3.Instance->CNT;
  motor_count_revolution_from_pulse(mot->pulse_count);
  htim3.Instance->CNT = 0;
}

void motor_count_revolution_from_pulse(uint16_t pulse)
{
  pulse += motor_count_store_rev_get();
  if(pulse > (ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGET1T2))
  {
    motor_pulse_revolution_set(motor_pulse_revolution_get() + 1);
    pulse -= (ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGET1T2);
  }
  motor_count_store_rev_set(pulse);
}

void motor_count_store_rev_set(uint16_t pulse)
{
  hMotor.pulse_store = pulse;
}
uint16_t motor_count_store_rev_get(void)
{
  return hMotor.pulse_store;
}

void motor_pulse_revolution_set(uint32_t revolution)
{
  hMotor.pulse_revolution = revolution;
}
uint32_t motor_pulse_revolution_get(void)
{
  return hMotor.pulse_revolution;
}

void motor_program_revolution_set(uint32_t revolution)
{
  hProgram.cycle_revolutions = revolution;
}
uint32_t motor_program_revolution_get(void)
{
  return hProgram.cycle_revolutions;
}

void motor_count_update(motorParam_t *mot)
{
  mot->pulse_count++;                             //for exti mode
}

void motor_dir_toggle(void)
{
  hMotor.ref_speed = -hMotor.ref_speed; //flip rotation direction
  pid_reset(&(hMotor.pid_regulator));
}

uint8_t motor_dir_get(void)
{
  return hMotor.direction;
}

void tim_pwmDutyCycleSet(TIM_HandleTypeDef *htim, float duty, uint8_t dir)
{

  /*Get Timer Period for proper duty cycle calculation*/
  uint16_t dutyCycle = tim_pwmPeriod_get(htim)*duty/100;
  /*Output Compare register update in "tim.c"*/
  tim_pwmPulse_set(htim->Instance, dutyCycle, dir);

}

void motor_pwmDutyCycle_set(TIM_HandleTypeDef *htim, float duty, uint8_t dir)
{
  tim_pwmDutyCycleSet(htim, duty, dir);
}

float motor_pwmDutyCycle_get(void)
{
  return hMotor.pwm_duty;
}

float motor_current_get(void)
{                          //ADC_Value/(OpAmp_Gain*ADC_Max)*(1/Shunt_resistor)*Ref_voltage
  float current = iir_filter_current(&filter, ADC_result[0]);
  hMotor.current = (current + 32.5)/216; //ADC[0]/(25*4095)*(1/0.002)*3.3
  if(hMotor.current > motor_safety_max_current_get())
    motor_fault_OC_protection_set(true);

  return hMotor.current;
}

float motor_voltage_get(void)//inverted voltage divider
{
  float voltage = ADC_result[1];

  voltage = voltage * 0.0111 + 0.1757; //ADC values plugged in Excel, compared to voltage from an adjustable power supply
/*  if(voltage < 2080)
  {
    voltage = (voltage+10)/160;
  }
  else
  {
    voltage = (voltage-1621)/65;
  }*/

  hMotor.voltage = voltage;

  if(hMotor.voltage > motor_safety_max_voltage_get())
    motor_fault_OV_protection_set(true);
  else if(hMotor.voltage < motor_safety_min_voltage_get())
    motor_fault_UV_protection_set(true);
  return hMotor.voltage;
}

float motor_torque_get(void)
{
  if(hMotor.speed > 1)
    hMotor.torque = (hMotor.efficiency * hMotor.pwm_duty/100 * hMotor.power) / hMotor.speed;
  else
    hMotor.torque = 0;

  return hMotor.torque;
}

uint8_t motor_load_get(void)
{
  return hMotor.load;
}

float motor_power_get(void)
{
  hMotor.power = hMotor.voltage*hMotor.current;
  return hMotor.power;
}

uint16_t motor_efficiency_get(void)
{
  return hMotor.efficiency;
}

/*TODO PID section*/
float pid_Kp_get(void)
{
  return hMotor.pid_regulator.Kp;
}
void pid_Kp_set(float value)
{
  hMotor.pid_regulator.Kp = value;
}

float pid_Ki_get(void)
{
  return hMotor.pid_regulator.Ki;
}
void pid_Ki_set(float value)
{
  hMotor.pid_regulator.Ki = value;
}

float pid_Kd_get(void)
{
  return hMotor.pid_regulator.Kd;
}
void pid_Kd_set(float value)
{
  hMotor.pid_regulator.Kd = value;
}

/*TODO Custom program section*/
void motor_program_cycle_ref_total_set(uint32_t cycle)
{
  hProgram.cycle_ref_total = cycle;
}
uint32_t motor_program_cycle_ref_total_get(void)
{
  return hProgram.cycle_ref_total;
}

void motor_program_cycle_total_set(uint32_t cycle)
{
  hProgram.cycle_total = cycle;
  GREEN_LED_OFF;
}
uint32_t motor_program_cycle_total_get(void)
{
  return hProgram.cycle_total;
}

void motor_program_cycleCount_set (uint32_t cycle)
{
  hProgram.cycle_count = cycle;
}
uint32_t motor_program_cycleCount_get (void)
{
  return hProgram.cycle_count;
}

void motor_program_ref_cycleCount_CW_set(uint16_t cycle)
{
  hProgram.cycle_ref_count_CW = cycle;
}
uint16_t motor_program_ref_cycleCount_CW_get(void)
{
  return hProgram.cycle_ref_count_CW;
}

void motor_program_cycleCount_CW_set (uint32_t cycle)
{
  hProgram.cycle_count_CW = cycle;
}
uint32_t motor_program_cycleCount_CW_get (void)
{
  return hProgram.cycle_count_CW;
}

void motor_program_ref_cycleCount_CCW_set(uint16_t cycle)
{
  hProgram.cycle_ref_count_CCW = cycle;
}
uint16_t motor_program_ref_cycleCount_CCW_get(void)
{
  return hProgram.cycle_ref_count_CCW;
}

void motor_program_cycleCount_CCW_set (uint32_t cycle)
{
  hProgram.cycle_count_CCW = cycle;
}
uint32_t motor_program_cycleCount_CCW_get (void)
{
  return hProgram.cycle_count_CCW;
}

void motor_program_cycleTime_CW_set (uint32_t time)
{
  hProgram.cycle_time_CW = time;
}
uint32_t motor_program_cycleTime_CW_get (void)
{
  return hProgram.cycle_time_CW;
}

void motor_program_cycleTime_CCW_set (uint32_t time)
{
  hProgram.cycle_time_CCW = time;
}
uint32_t motor_program_cycleTime_CCW_get (void)
{
  return hProgram.cycle_time_CCW;
}

void motor_program_cycleTime_idle_set (uint32_t time)
{
  hProgram.cycle_time_idle = time;
}
uint32_t motor_program_cycleTime_idle_get (void)
{
  return hProgram.cycle_time_idle;
}

void motor_program_ramp_up_set (float ramp)
{
  if((ramp <= hSafety.safety_max_ramp_up) && (ramp > 0))
    hProgram.ramp_up = ramp;
}
float motor_program_ramp_up_get (void)
{
  return hProgram.ramp_up;
}

void motor_program_ramp_down_set (float ramp)
{
  if((ramp <= hSafety.safety_max_ramp_down) && (ramp > 0))
    hProgram.ramp_down = ramp;
}
float motor_program_ramp_down_get (void)
{
  return hProgram.ramp_down;
}

/*TODO Safety functions section*/

void motor_safety_fault_trigger_set(void)
{
  uint8_t temperature_fault = 0;
  for (uint8_t i = 0; i < sizeof(hFault.safety_OT_protection); i++)
    temperature_fault += motor_fault_OT_protection_get(i);

  if((motor_fault_OC_protection_get() == true)
     || (motor_fault_OV_protection_get() == true)
     || (motor_fault_UV_protection_get() == true)
     || (motor_fault_encoder_fail_get() == true)
     || (temperature_fault > 0))
  {
    hFault.safety_fault_triggered = true;
    RED_LED_ON;
  }
}

bool motor_safety_fault_trigger_get(void)
{
  return hFault.safety_fault_triggered;
}

void motor_safety_fault_trigger_reset(void)
{
  hFault.safety_fault_triggered = false;
  RED_LED_OFF;
}

void motor_safety_temp_threshold_set(uint8_t sensor, float value)
{
  hSafety.safety_temp_threshold[sensor] = value;
}
float motor_safety_temp_threshold_get(uint8_t sensor)
{
  return hSafety.safety_temp_threshold[sensor];
}

void motor_safety_max_rpm_set(float value)
{
  if((value <= MOTOR_SAFETY_RPM_MAX) && (value >= -MOTOR_SAFETY_RPM_MAX))
    hSafety.safety_max_rpm = value;
}
float motor_safety_max_rpm_get(void)
{
  return hSafety.safety_max_rpm;
}

void motor_safety_max_ramp_up_set(float value)
{
  if((value <= MOTOR_SAFETY_RAMP_UP_MAX) && (value > 0))
    hSafety.safety_max_ramp_up = value;
}
float motor_safety_max_ramp_up_get(void)
{
  return hSafety.safety_max_ramp_up;
}

void motor_safety_max_ramp_down_set(float value)
{
  if((value <= MOTOR_SAFETY_RAMP_DOWN_MAX) && (value > 0))
    hSafety.safety_max_ramp_down = value;
}
float motor_safety_max_ramp_down_get(void)
{
  return hSafety.safety_max_ramp_down;
}

void motor_safety_max_voltage_set(float value)
{
  if((value <= MOTOR_SAFETY_OV_MAX) && (value >= hSafety.safety_min_voltage))
    hSafety.safety_max_voltage = value;
}
float motor_safety_max_voltage_get(void)
{
  return hSafety.safety_max_voltage;
}

void motor_safety_min_voltage_set(float value)
{
  if((value >= MOTOR_SAFETY_UV_MIN) && (value <= hSafety.safety_max_voltage))
    hSafety.safety_min_voltage = value;
}
float motor_safety_min_voltage_get(void)
{
  return hSafety.safety_min_voltage;
}

void motor_safety_max_current_set(float value)
{
  if((value <= MOTOR_SAFETY_OC_MAX) && (value > 0))
    hSafety.safety_max_current = value;
}
float motor_safety_max_current_get(void)
{
  return hSafety.safety_max_current;
}

/*TODO Fault function section*/
void motor_faults_reset(void)
{
  motor_fault_OC_protection_set(false);
  motor_fault_OV_protection_set(false);
  motor_fault_UV_protection_set(false);
  motor_fault_encoder_fail_set(false);
  for(uint8_t i = 0; i < sizeof(hFault.safety_OT_protection); i++)
    hFault.safety_OT_protection[i] = false;
  motor_safety_fault_trigger_reset();
}

void motor_fault_OC_protection_set(bool state)//over current
{
  if(state == true || state == false)
    hFault.safety_OC_protection = state;
}
bool motor_fault_OC_protection_get(void)
{
  return hFault.safety_OC_protection;
}

void motor_fault_OV_protection_set(bool state)//over voltage
{
  if(state == true || state == false)
    hFault.safety_OV_protection = state;
}
bool motor_fault_OV_protection_get(void)
{
  return hFault.safety_OV_protection;
}

void motor_fault_UV_protection_set(bool state)//under voltage
{
  if(state == true || state == false)
    hFault.safety_UV_protection = state;
}
bool motor_fault_UV_protection_get(void)
{
  return hFault.safety_UV_protection;
}

void motor_fault_encoder_fail_set(bool state)//encoder fail
{
  if(state == true || state == false)
    hFault.safety_encoder_fail = state;
}
bool motor_fault_encoder_fail_get(void)
{
  return hFault.safety_encoder_fail;
}

void      motor_fault_OT_protection_set(uint8_t sensor, bool state)
{
  if(state == true || state == false)
    hFault.safety_OT_protection[sensor] = state;
}

bool      motor_fault_OT_protection_get(uint8_t sensor)
{
  return hFault.safety_OT_protection[sensor];
}











/********************************************************/
