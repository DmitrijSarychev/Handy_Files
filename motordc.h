/*
 * motordc.h
 *
 *  Created on: Feb 22, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_MOTORDC_H_
#define INC_MOTORDC_H_

#include "main.h"
#include "pid_regulator.h"
#include "stdbool.h"

#define PWM_RIGHT_ON      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2))
#define PWM_RIGHT_OFF     (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2))
#define PWM_LEFT_ON       (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3))
#define PWM_LEFT_OFF      (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3))
#define LOW_PWM_RIGHT_ON  (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2))
#define LOW_PWM_RIGHT_OFF (HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2))
#define LOW_PWM_LEFT_ON   (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3))
#define LOW_PWM_LEFT_OFF  (HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3))

#define GREEN_LED_ON      (HAL_GPIO_WritePin(ST_LED_GREEN_GPIO_Port, ST_LED_GREEN_Pin, GPIO_PIN_SET))
#define GREEN_LED_OFF     (HAL_GPIO_WritePin(ST_LED_GREEN_GPIO_Port, ST_LED_GREEN_Pin, GPIO_PIN_RESET))

#define RED_LED_ON        (HAL_GPIO_WritePin(ST_LED_RED_GPIO_Port, ST_LED_RED_Pin, GPIO_PIN_SET))
#define RED_LED_OFF       (HAL_GPIO_WritePin(ST_LED_RED_GPIO_Port, ST_LED_RED_Pin, GPIO_PIN_RESET))

#define TEMP_SENS_VOLT_DIV  10000; //10k
#define DEFAULT_DUTY_CYCLE  0.5f;
#define SHUNT_RESISTANCE 500 //0.002mOh

#define MOTOR_Kp 0.006f
#define MOTOR_Ki 0.0002f
#define MOTOR_Kd 0.003f
#define MOTOR_ANTI_WINDUP 100.0f
#define AVG_FILTER_STAGE 5
#define PWM_DUTY_MAX 100
#define RECT_DUTY_MIN 1     //Active rectification for bypassing MOSFET body diode

#define MOTOR_SAFETY_RPM_MAX  3000.0f
#define MOTOR_SAFETY_RAMP_DOWN_MAX 1350.0f  //[rpm/s]
#define MOTOR_SAFETY_RAMP_UP_MAX 5000.0f    //[rpm/s]
#define MOTOR_SAFETY_OC_MAX 20.0f           //[A]
#define MOTOR_SAFETY_OV_MAX 30.0f           //[V]
#define MOTOR_SAFETY_UV_MIN 15.0f           //[V]
#define MOTOR_SAFETY_TEMP_MAX 80.0f         //[C]
#define MOTOR_SAFETY_ENCODER_FAIL_THRESHOLD 0.20f   //[A]

#define CW 0
#define CCW 1

#define ON 1
#define OFF 0

#define AUTO 1
#define MANUAL 0

#define ENCODER_RESOLUTION 2000
#define TIMER_CONF_BOTH_EDGET1T2 2
#define GEAR_REDUCTION 1
#define TIMER_FREQ 20
#define MINUTE 60

#define UI_HEADER_ROW   1
#define UI_HEADER_COL   2

#define UI_DATA_ROW   3
#define UI_DATA_COL   17

#define PID_FLASH_PAGE_ADDRESS 0x0000
#define SAFETY_FLASH_PAGE_ADDRESS 0x0100
#define PROGRAM_FLASH_PAGE_ADDRESS 0x0200

typedef struct
{
  //TIM_HandleTypeDef *timer; //Pass the handler of the timer responsible for counting RPM
  uint8_t   program;      //selected test program
  uint8_t   program_run;  //Run/stop program in auto
  uint8_t   status;       //Running/Stop
  uint8_t   mode;         //Auto/Manual
  float     ref_speed;    //set speed/ reference speed
  float     ref_speed_ramp; //the actual set speed after applying ramp_down (TIM10)
  uint16_t  resolution;   //encoder
  uint16_t  pulse_store;  //stores the pulse_count value here and resets it (timer based); Circular buffer
  uint16_t  pulse_revolution;  //Motor revolutions elapsed
  uint16_t  pulse_count;  //pulses from the encoder
  float     speed;        //actual speed
  uint8_t   direction;    //0=CW; 1=CCW
  float     pwm_duty;     //%0-100% PWM duty cycle
  float     current;
  float     voltage;
  float     torque;
  uint8_t   load;         //0-100% motor load
  float     power;        //electric power
  float     efficiency;   //probably a constant?
  pid_t     pid_regulator;//PID struct handler

}motorParam_t;

typedef struct
{
  float     safety_temp_threshold[6];//max temperature threshold for thermistors
  float     safety_max_rpm;          //max physical
  float     safety_max_ramp_up;      //minimal spin-up/down ramp so the transistors don't blow up with big induction load
  float     safety_max_ramp_down;    //minimal spin-up/down ramp so the transistors don't blow up with big induction load
  float     safety_max_voltage;      //shut-down if exceeded
  float     safety_min_voltage;      //shut-down if too low voltage
  float     safety_max_current;      //shut-down if exceeded (or try to reduce PWM duty TODO)
}__attribute__((packed)) motorSafety_t; //this one is saved to external FLASH

typedef struct
{
  bool      safety_OC_protection;    //Trigger over current protection
  bool      safety_OV_protection;    //Trigger over voltage protection
  bool      safety_UV_protection;    //Trigger under voltage protection
  bool      safety_OT_protection[5]; //Trigger over temperature protection
  bool      safety_encoder_fail;     //Trigger if current present at 0 rpm
  bool      safety_fault_triggered;
}motorFault_t;

typedef struct
{
  uint32_t  cycle_ref_total;    //Program total cycles reference value (cycle_cout*(cycle_count_CW+cycle_count_CCW)
  uint32_t  cycle_total;        //Program total cycles elapsed
  uint32_t  cycle_count;        //Program cycle input value by user, reference value
  uint16_t  cycle_ref_count_CW; //Program reference value of cycles before direction change
  uint32_t  cycle_count_CW;     //Program cycle how many cycles passed in CW direction
  uint16_t  cycle_ref_count_CCW; //Program reference value of cycles before direction change
  uint32_t  cycle_count_CCW;    //Program cycle how many cycles passed in CCW direction
  uint32_t  cycle_time_CW;      //Program cycle time CW direction [ms]
  uint32_t  cycle_time_CCW;     //Program cycle time CCW direction [ms]
  uint32_t  cycle_time_idle;    //Program cysle time idle (motor off) [ms]
  uint32_t  cycle_revolutions;  //Program revolutions per cycle
  float     ramp_down;          //Program deceleration ramp [obr/s^2]
  float     ramp_up;            //Program acceleration ramp [obr/s^2]

}__attribute__((packed)) motorProgram_t; //this one is saved to external FLASH

typedef enum
{
  MOTOR_PROGRAM_1 = 0,
  MOTOR_PROGRAM_2,
  MOTOR_PROGRAM_3
}motorProgram_e;

typedef enum
{
  TEMPERATURE_1 = 0,
  TEMPERATURE_2,
  TEMPERATURE_3,
  TEMPERATURE_4,
  TEMPERATURE_5
}motorTemperature_e;

extern motorParam_t hMotor;
extern motorSafety_t hSafety;
extern motorProgram_t hProgram;
extern motorFault_t hFault;

/* Motor settings and parameters */
void      motor_init(motorParam_t *mot/*, TIM_HandleTypeDef *tim*/);


HAL_StatusTypeDef motor_program_init_from_flash (motorProgram_t *program);
void      motor_program_init(motorProgram_t *program);

HAL_StatusTypeDef motor_safety_init_from_flash (motorSafety_t *safety);
void      motor_safety_init(motorSafety_t *safety);

void      motor_reset(motorParam_t *mot);

void      motor_program_set(motorParam_t *mot, uint8_t program);
uint8_t   motor_program_get(void);

void      motor_program_run_toggle(motorParam_t *mot);
void      motor_program_run_set(uint8_t status);
uint8_t   motor_program_run_get(void);

void      motor_status_toggle(void);
uint8_t   motor_status_get(void);
void      motor_status_set(uint8_t status);

void      motor_mode_set(uint8_t value);
uint8_t   motor_mode_get(void);

void      motor_refSpeed_set(motorParam_t *mot, float speed);
float     motor_refSpeed_get(void);

float     motor_speed_get(void);
void      motor_speed_update(motorParam_t *mot, motorProgram_t *program);

void      motor_count_store(motorParam_t *mot);
void      motor_count_revolution_from_pulse(uint16_t pulse);

void      motor_count_store_rev_set(uint16_t pulse);
uint16_t  motor_count_store_rev_get(void);

void      motor_pulse_revolution_set(uint32_t revolution);
uint32_t  motor_pulse_revolution_get(void);

void      motor_program_revolution_set(uint32_t revolution);
uint32_t  motor_program_revolution_get(void);

void      motor_count_update(motorParam_t *mot);

void      motor_dir_toggle(void);
uint8_t   motor_dir_get(void);

void      tim_pwmDutyCycleSet(TIM_HandleTypeDef *htim, float duty, uint8_t dir);
void      motor_pwmDutyCycle_set(TIM_HandleTypeDef *htim, float duty, uint8_t dir);
float     motor_pwmDutyCycle_get(void);

float     motor_current_get(void);
float     motor_voltage_get(void);

float     motor_torque_get(void);
uint8_t   motor_load_get(void);
float     motor_power_get(void);
uint16_t  motor_efficiency_get(void);

float     pid_Kp_get(void);
void      pid_Kp_set(float value);

float     pid_Ki_get(void);
void      pid_Ki_set(float value);

float     pid_Kd_get(void);
void      pid_Kd_set(float value);

void      motor_program_cycle_ref_total_set(uint32_t cycle);
uint32_t  motor_program_cycle_ref_total_get(void);

void      motor_program_cycle_total_set(uint32_t cycle);
uint32_t  motor_program_cycle_total_get(void);

void      motor_program_cycleCount_set(uint32_t cycle);
uint32_t  motor_program_cycleCount_get(void);

void      motor_program_ref_cycleCount_CW_set(uint16_t cycle);
uint16_t   motor_program_ref_cycleCount_CW_get(void);

void      motor_program_cycleCount_CW_set(uint32_t cycle);
uint32_t  motor_program_cycleCount_CW_get(void);

void      motor_program_ref_cycleCount_CCW_set(uint16_t cycle);
uint16_t   motor_program_ref_cycleCount_CCW_get(void);

void      motor_program_cycleCount_CCW_set(uint32_t cycle);
uint32_t  motor_program_cycleCount_CCW_get(void);

void      motor_program_cycleTime_CW_set(uint32_t time);
uint32_t  motor_program_cycleTime_CW_get(void);

void      motor_program_cycleTime_CCW_set(uint32_t time);
uint32_t  motor_program_cycleTime_CCW_get(void);

void      motor_program_cycleTime_idle_set(uint32_t time);
uint32_t  motor_program_cycleTime_idle_get(void);

void      motor_program_ramp_up_set(float ramp);
float     motor_program_ramp_up_get(void);

void      motor_program_ramp_down_set(float ramp);
float     motor_program_ramp_down_get(void);

void      motor_safety_fault_trigger_set(void);
bool      motor_safety_fault_trigger_get(void);

void      motor_safety_fault_trigger_reset(void);

void      motor_safety_temp_threshold_set(uint8_t sensor, float value);
float     motor_safety_temp_threshold_get(uint8_t sensor);

void      motor_safety_max_rpm_set(float value);
float     motor_safety_max_rpm_get(void);

void      motor_safety_max_ramp_up_set(float value);
float     motor_safety_max_ramp_up_get(void);

void      motor_safety_max_ramp_down_set(float value);
float     motor_safety_max_ramp_down_get(void);

void      motor_safety_max_voltage_set(float value);
float     motor_safety_max_voltage_get(void);

void      motor_safety_min_voltage_set(float value);
float     motor_safety_min_voltage_get(void);

void      motor_safety_max_current_set(float value);
float     motor_safety_max_current_get(void);

void      motor_faults_reset(void);

void      motor_fault_OC_protection_set(bool state);//over current
bool      motor_fault_OC_protection_get(void);

void      motor_fault_OV_protection_set(bool state);//over voltage
bool      motor_fault_OV_protection_get(void);

void      motor_fault_UV_protection_set(bool state);//under voltage
bool      motor_fault_UV_protection_get(void);

void      motor_fault_encoder_fail_set(bool state);//encoder fail
bool      motor_fault_encoder_fail_get(void);

void      motor_fault_OT_protection_set(uint8_t sensor, bool state);
bool      motor_fault_OT_protection_get(uint8_t sensor);

#endif /* INC_MOTORDC_H_ */
