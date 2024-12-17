/*
 * interface.h
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include "main.h"
#include "temperature.h"
#include "motordc.h"

#define USB_NUMBER_BUFFER_LENGTH 6

#define CMD_EX(Code, Cmd) (((Code)<<8) | (Cmd)) //Move byte left
#define CAPITALIZE_CHAR(Cmd) (((Cmd) >= 'a' && (Cmd) <= 'z') ? ((Cmd) & 0xDF) : (Cmd)) //Capitalize character

#define UI_ENTER_DATA_TEXT_ROW 50
#define UI_ENTER_DATA_TEXT_COL 2

#define UI_PID_ENTER_DATA_TEXT_ROW 8
#define UI_PID_ENTER_DATA_TEXT_COL 1
#define UI_PID_ENTER_COL_OFFSET    17

#define UI_TIME_ENTER_DATA_TEXT_ROW 8
#define UI_TIME_ENTER_DATA_TEXT_COL 1
#define UI_TIME_ENTER_COL_OFFSET    17

#define UI_DATE_ENTER_DATA_TEXT_ROW 8
#define UI_DATE_ENTER_DATA_TEXT_COL 1
#define UI_DATE_ENTER_COL_OFFSET    17

#define UI_PROG_ENTER_DATA_TEXT_ROW 14
#define UI_PROG_ENTER_DATA_TEXT_COL 1
#define UI_PROG_ENTER_COL_OFFSET    20

#define UI_SAFETY_ENTER_DATA_TEXT_ROW 16
#define UI_SAFETY_ENTER_DATA_TEXT_COL 1
#define UI_SAFETY_ENTER_COL_OFFSET    20

#define UI_SAFETY_DISPLAY_DATA_TEXT_ROW 39
#define UI_SAFETY_DISPLAY_DATA_TEXT_COL 30

#define UI_ENTER_DATA_ROW 50
#define UI_ENTER_DATA_COL 20

typedef struct
{
  uint8_t        menu_submenu; //How deep in the submenu are you
  uint8_t        menu_row;
  uint8_t        menu_col;
  uint8_t        logMode;
  motorParam_t   *motor;
  temperature_t  temp_c;
  motorSafety_t  *safety;
  motorProgram_t *program;
  motorFault_t   *fault;
}displayData_t;

typedef enum
{
  DISPLAY_SET_NOTHING = 0,
  DISPLAY_SET_MOTOR_SPEED,
  DISPLAY_SET_PROGRAM,
  DISPLAY_SET_PROGRAM_EDIT,
  DISPLAY_SET_PID,
  DISPLAY_SET_TIME,
  DISPLAY_SET_DATE,
  DISPLAY_SET_SAFETY,
  DISPLAY_SET_CONFIG
}displayMenu_e;

typedef enum
{
  DISPLAY_CONFIG_TIME = 0,
  DISPLAY_CONFIG_DATE,
  DISPLAY_CONFIG_PID,
  DISPLAY_CONFIG_PROGRAM_EDIT,
  DISPLAY_CONFIG_SAFETY,
  DISPLAY_CONFIG_MAIN/*This has to be last, because it determines the size of this enum!*/
}displayConfig_e;

typedef enum
{
  DISPLAY_PID_KP = 0,
  DISPLAY_PID_KI,
  DISPLAY_PID_KD/*This has to be last, because it determines the size of this enum!*/
}displayPID_e;

typedef enum
{
  DISPLAY_PROGRAM_NUMBER_CYCLES = 0,
  DISPLAY_PROGRAM_CW_CYCLES,
  DISPLAY_PROGRAM_CCW_CYCLES,
  DISPLAY_PROGRAM_CYCLE_DURATION, /*revolution count*/
  DISPLAY_PROGRAM_CYCLE_TIME_IDLE,
  DISPLAY_PROGRAM_RAMP_UP,
  DISPLAY_PROGRAM_RAMP_DOWN/*This has to be last, because it determines the size of this enum!*/
}displayProgram_e;

typedef enum
{
  DATA_MODE = 0,
  LOG_MODE,
  CONFIG_MODE
}displayMenuMode_e;

typedef enum
{
  TIME_HOUR = 0,
  TIME_MINUTE,
  TIME_SECOND/*This has to be last, because it determines the size of this enum!*/
}displayTime_e;

typedef enum
{
  DATE_YEAR = 0,
  DATE_MONTH,
  DATE_DAY/*This has to be last, because it determines the size of this enum!*/
}displayDate_e;

typedef enum
{
  DISPLAY_SAFETY_CURRENT_MAX = 0,
  DISPLAY_SAFETY_VOLTAGE_MAX,
  DISPLAY_SAFETY_VOLTAGE_MIN,
  DISPLAY_SAFETY_RAMP_UP_MAX,
  DISPLAY_SAFETY_RAMP_DOWN_MAX,
  DISPLAY_SAFETY_RPM_MAX,
  /*Any addition to this ENUM must go here, otherwise risk compilation errors and improper variable handling*/

  /*No tresspassing hence forth*/
  DISPLAY_SAFETY_TEMP1,/*TEMP1-TEMP5 need to be grouped together to avoid sensor selection errors*/
  DISPLAY_SAFETY_TEMP2,
  DISPLAY_SAFETY_TEMP3,
  DISPLAY_SAFETY_TEMP4,
  DISPLAY_SAFETY_TEMP5,/*This has to be last, because it determines the size of this enum!*/
}displaySafety_e;

extern displayData_t hdisplay;

void interface_init(displayData_t * hData, motorParam_t *mot, motorProgram_t *program, motorSafety_t *safety);
void interface_save_to_flash(void);

void interface_refMotorSpeed_set(void);
void interface_program_elect(void);

void interface_kp_set(void);
void interface_ki_set(void);
void interface_kd_set(void);

void interface_timeHour_set(void);
void interface_timeMinute_set(void);
void interface_timeSecond_set(void);

void interface_dateYear_set(void);
void interface_dateMonth_set(void);
void interface_dateDay_set(void);
void interface_config_set(void);

void interface_program_cycleCount_set (void);
void interface_program_cycleCount_CW_set (void);
void interface_program_cycleCount_CCW_set (void);
void interface_program_cycleTime_CW_set (void);
void interface_program_cycleTime_CCW_set (void);
void interface_program_cycleTime_idle_set (void);
void interface_program_ramp_up_set (void);
void interface_program_ramp_down_set (void);

void interface_safety_max_current_set(void);
void interface_safety_min_voltage_set(void);
void interface_safety_max_voltage_set(void);
void interface_safety_max_ramp_up_set(void);
void interface_safety_max_ramp_down_set(void);
void interface_safety_max_rpm_set(void);
void interface_safety_temp_set(uint8_t sensor);

/*Terminal commands*/
void USB_receive(void); // function for data probing, use with RTOS or in while loop
void USB_receiveBuffer(uint8_t *Buf, uint32_t *Len);
void USB_receiveNumber(void);
void USB_receiveCommand(uint8_t *command);
void clearUSB_receiveBuffer(void);

void interface_reset(void);
void interface_number_update(void);
void interface_UpdateUI(void);
void interface_displayData(displayData_t *hData);
int32_t ComTx_Printf(char const *format, ... );
void interface_updateData(displayData_t *hData);
void interface_logMode_toggle(void);

#endif /* INC_INTERFACE_H_ */
