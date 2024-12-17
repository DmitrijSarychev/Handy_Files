/*
 * interface.c
 *
 *  Created on: Mar 25, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */


#include "motordc.h"
#include "interface.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <terminal.h>
#include "math.h"
#include "stdbool.h"
#include "iir_filter.h"
#include "temperature.h"
#include "rtc.h"
#include "MX25L4006EM_FLASH.h"

uint8_t         UsbRxData[5];
char            UsbNumber_buffer[USB_NUMBER_BUFFER_LENGTH];
bool            USBreceiveFlag = false;
bool            USBreceiveNumber = false;
bool            USBlogModeRun = false;


displayMenu_e selectMenu;
displayPID_e selectPID;
displayData_t hdisplay;
displayMenuMode_e selectMenuMode;
displayTime_e selectTime;
displayDate_e selectDate;
displayConfig_e selectConfig;
displayProgram_e selectProgram_display;
displaySafety_e selectSafety;

RTC_TimeTypeDef htime;
RTC_DateTypeDef hdate;

void interface_init(displayData_t * hData, motorParam_t *mot, motorProgram_t *program, motorSafety_t *safety)
{
  interface_reset();
  hData->motor = mot;
  hData->program = program;
  hData->safety = safety;
  hData->logMode = 0;
}

void interface_save_to_flash(void)
{
  /*SAVE ALL DATA TO FLASH BEGIN*/
  MX25L4006EM_SectorErase(0);
  uint8_t* write_data = (uint8_t*)&hMotor.pid_regulator;
  MX25L4006EM_PageProgram(PID_FLASH_PAGE_ADDRESS, write_data, sizeof(hMotor.pid_regulator));
  write_data = (uint8_t*)&hSafety;
  MX25L4006EM_PageProgram(SAFETY_FLASH_PAGE_ADDRESS, write_data, sizeof(hSafety));
  write_data = (uint8_t*)&hProgram;
  MX25L4006EM_PageProgram(PROGRAM_FLASH_PAGE_ADDRESS, write_data, sizeof(hProgram));
  /*SAVE ALL DATA TO FLASH END*/
}

void interface_refMotorSpeed_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Speed [RPM]:  ");
  USBreceiveNumber = true;
}

void interface_program_select(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Sel. program: ");
  USBreceiveNumber = true;
}
void interface_kp_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter KP:     ");
  selectPID = DISPLAY_PID_KP;
  USBreceiveNumber = true;
}
void interface_ki_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter KI:     ");
  selectPID = DISPLAY_PID_KI;
  USBreceiveNumber = true;
}
void interface_kd_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter KD:     ");
  selectPID = DISPLAY_PID_KD;
  USBreceiveNumber = true;
}
/*RTC Date and Time settings*/
void interface_timeHour_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter hour:   ");
  selectTime = TIME_HOUR;
  USBreceiveNumber = true;
}
void interface_timeMinute_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter minute: ");
  selectTime = TIME_MINUTE;
  USBreceiveNumber = true;
}
void interface_timeSecond_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter second: ");
  selectTime = TIME_SECOND;
  USBreceiveNumber = true;
}

void interface_dateYear_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter year:   ");
  selectDate = DATE_YEAR;
  USBreceiveNumber = true;
}
void interface_dateMonth_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter month:  ");
  selectDate = DATE_MONTH;
  USBreceiveNumber = true;
}
void interface_dateDay_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PID_ENTER_DATA_TEXT_ROW;
    col = UI_PID_ENTER_DATA_TEXT_COL;
  }
                                                               //11111
                                                      //12345678901234
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter day:    ");
  selectDate = DATE_DAY;
  USBreceiveNumber = true;
}

void interface_logMode_toggle(void)
{
  interface_reset();
  if(selectMenuMode <= 1)
    selectMenuMode ^= 1;
  else
    selectMenuMode = LOG_MODE;
  interface_UpdateUI();
}

void interface_config_set(void)
{
  interface_reset();
  selectConfig = DISPLAY_CONFIG_MAIN;
  selectMenu = DISPLAY_SET_CONFIG;
  selectMenuMode = CONFIG_MODE;
  hdisplay.menu_row = 0;
  interface_UpdateUI();
}

void interface_program_cycleCount_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //111111111
                                                      //123456789012345678
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter NCycle:     ");
  selectProgram_display = DISPLAY_PROGRAM_NUMBER_CYCLES;
  USBreceiveNumber = true;
}
void interface_program_cycleCount_CW_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter CW Cycle:     ");
  selectProgram_display = DISPLAY_PROGRAM_CW_CYCLES;
  USBreceiveNumber = true;
}
void interface_program_cycleCount_CCW_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter CCW Cycle:     ");
  selectProgram_display = DISPLAY_PROGRAM_CCW_CYCLES;
  USBreceiveNumber = true;
}
void interface_program_cycleTime_CW_set(void)//TODO
{

}
void interface_program_cycleTime_CCW_set(void)//TODO Set CCW cycle time
{

}
void interface_program_cycleTime_idle_set(void)//TODO Set idle cycle time
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter Idle time:");
  selectProgram_display = DISPLAY_PROGRAM_CYCLE_TIME_IDLE;
  USBreceiveNumber = true;
}

void interface_program_revolution_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter Cyc. dur.:     ");
  selectProgram_display = DISPLAY_PROGRAM_CYCLE_DURATION;
  USBreceiveNumber = true;
}

void interface_program_ramp_up_set (void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter ramp:     ");
  selectProgram_display = DISPLAY_PROGRAM_RAMP_UP;
  USBreceiveNumber = true;
}

void interface_program_ramp_down_set (void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_PROG_ENTER_DATA_TEXT_ROW;
    col = UI_PROG_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter ramp:     ");
  selectProgram_display = DISPLAY_PROGRAM_RAMP_DOWN;
  USBreceiveNumber = true;
}

void interface_safety_max_current_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX Current:  ");
  selectSafety = DISPLAY_SAFETY_CURRENT_MAX;
  USBreceiveNumber = true;
}
void interface_safety_min_voltage_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MIN Voltage:  ");
  selectSafety = DISPLAY_SAFETY_VOLTAGE_MIN;
  USBreceiveNumber = true;
}
void interface_safety_max_voltage_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX Voltage:  ");
  selectSafety = DISPLAY_SAFETY_VOLTAGE_MAX;
  USBreceiveNumber = true;
}

void interface_safety_max_ramp_up_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX Ramp:     ");
  selectSafety = DISPLAY_SAFETY_RAMP_UP_MAX;
  USBreceiveNumber = true;
}

void interface_safety_max_ramp_down_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX Ramp:     ");
  selectSafety = DISPLAY_SAFETY_RAMP_DOWN_MAX;
  USBreceiveNumber = true;
}
void interface_safety_max_rpm_set(void)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX RPM:      ");
  selectSafety = DISPLAY_SAFETY_RPM_MAX;
  USBreceiveNumber = true;
}
void interface_safety_temp_set(uint8_t sensor)
{
  uint8_t row = /*UI_DATA_ROW*/UI_ENTER_DATA_TEXT_ROW;
  uint8_t col = /*UI_DATA_COL*/UI_ENTER_DATA_TEXT_COL;

  if(selectMenuMode == CONFIG_MODE)
  {
    row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
    col = UI_SAFETY_ENTER_DATA_TEXT_COL;
  }
                                                               //11111111112
                                                      //12345678901234567890
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Enter MAX Temp%d:  ", sensor + 1);
  selectSafety = DISPLAY_SAFETY_TEMP1 + sensor;
  USBreceiveNumber = true;
}

/*Terminal commands*/

/*This function is called every time a char is sent through the terminal.
Execution is called inside the usb_cdc_if.c file*/
void USB_receiveBuffer(uint8_t *Buf, uint32_t *Len)
{
  clearUSB_receiveBuffer();
  memcpy(UsbRxData, Buf, (*Len));
  USBreceiveFlag = true;
}

//This function is called cyclically in the main.c file
void USB_receive(void)
{
  uint8_t row = UI_ENTER_DATA_ROW;
  uint8_t col = UI_ENTER_DATA_COL;
  bool dot = false; //there is no dot
  if (USBreceiveFlag == true)
  {
    static uint8_t i = 0;         //cyclic buffer iteration
/*    ////////////////////BACKSPACE BEGIN*/
    if ((UsbRxData[0] == '\b') && (i > 0))
    {
      i--;
      UsbNumber_buffer[i] = '\0';
      switch(selectMenuMode)
      {
        case DATA_MODE:
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,UsbNumber_buffer);

          break;//DATA_MODE

        case LOG_MODE:

          break;//LOG_MODE

        case CONFIG_MODE:
          switch(selectConfig)
          {
            case DISPLAY_CONFIG_TIME:
                row = UI_TIME_ENTER_DATA_TEXT_ROW;
                col = UI_TIME_ENTER_DATA_TEXT_COL + UI_TIME_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_TIME

            case DISPLAY_CONFIG_DATE:
                row = UI_DATE_ENTER_DATA_TEXT_ROW;
                col = UI_DATE_ENTER_DATA_TEXT_COL + UI_DATE_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_DATE

            case DISPLAY_CONFIG_PID:
                row = UI_PID_ENTER_DATA_TEXT_ROW;
                col = UI_PID_ENTER_DATA_TEXT_COL + UI_PID_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_PID

            case DISPLAY_CONFIG_PROGRAM_EDIT:
                row = UI_PROG_ENTER_DATA_TEXT_ROW;
                col = UI_PROG_ENTER_DATA_TEXT_COL + UI_PROG_ENTER_COL_OFFSET;
              break;

            case DISPLAY_CONFIG_MAIN:
              break;//DISPLAY_CONFIG_MAIN

            case DISPLAY_CONFIG_SAFETY:
              row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
              col = UI_SAFETY_ENTER_DATA_TEXT_COL + UI_SAFETY_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_SAFETY
          }
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,UsbNumber_buffer);
          break;//CONFIG_MODE
      }//selectMenuMode
    }
/*    ////////////////////BACKSPACE END*/

/*    ////////////////////RETURN BEGIN*/
    else if ((UsbRxData[0] == '\r') /*&& (USBreceiveNumber == true)*/)  //TODO Press 'ENTER' to confirm value
    {
      float inputValue = 0;

      if(USBreceiveNumber == true)
      {
        sscanf(UsbNumber_buffer,"%f",&inputValue); //convert char array to int
      }
      switch(selectMenu)
      {
        case DISPLAY_SET_NOTHING:

          break;//DISPLAY_DO_NOTHING
        case DISPLAY_SET_MOTOR_SPEED: //Set reference speed
          motor_refSpeed_set(&hMotor, inputValue);
          interface_number_update();
          break;//DISPLAY_SET_MOTOR_SPEED

        case DISPLAY_SET_PROGRAM:  //Program selection
          if(inputValue > 0 && inputValue <=3)
            motor_program_set(&hMotor, (uint8_t)inputValue - 1);

          selectMenu = DISPLAY_SET_CONFIG;
          USBreceiveNumber = false;
          ComTx_VT100_ClearScreen();
          interface_UpdateUI();
          break;//DISPLAY_PROGRAM_SELECT

        case DISPLAY_SET_PROGRAM_EDIT: //TODO Program edit
          switch(selectProgram_display)
          {
            case DISPLAY_PROGRAM_NUMBER_CYCLES:
              motor_program_cycleCount_set(inputValue);
              motor_program_cycle_ref_total_set(motor_program_cycleCount_get()*(motor_program_ref_cycleCount_CW_get() + motor_program_ref_cycleCount_CCW_get()));
              break;//DISPLAY_PROGRAM_NUMBER_CYCLES

            case DISPLAY_PROGRAM_CW_CYCLES:
              motor_program_ref_cycleCount_CW_set(inputValue);
              motor_program_cycle_ref_total_set(motor_program_cycleCount_get()*(motor_program_ref_cycleCount_CW_get() + motor_program_ref_cycleCount_CCW_get()));
              break;//DISPLAY_PROGRAM_CW_CYCLES

            case DISPLAY_PROGRAM_CCW_CYCLES:
              motor_program_ref_cycleCount_CCW_set(inputValue);
              motor_program_cycle_ref_total_set(motor_program_cycleCount_get()*(motor_program_ref_cycleCount_CW_get() + motor_program_ref_cycleCount_CCW_get()));
              break;//DISPLAY_PROGRAM_CCW_CYCLES

            case DISPLAY_PROGRAM_CYCLE_DURATION:
              motor_program_revolution_set(inputValue);
              break;//DISPLAY_PROGRAM_CYCLE_DURATION

            case DISPLAY_PROGRAM_CYCLE_TIME_IDLE:
              motor_program_cycleTime_idle_set(inputValue);
              break;//DISPLAY_PROGRAM_CYCLE_TIME_IDLE

            case DISPLAY_PROGRAM_RAMP_UP:
              motor_program_ramp_up_set(inputValue);
              break;//DISPLAY_PROGRAM_RAMP_UP

            case DISPLAY_PROGRAM_RAMP_DOWN:
              motor_program_ramp_down_set(inputValue);
              break;//DISPLAY_PROGRAM_RAMP_DOWN
          }
          interface_save_to_flash();
          selectMenu = DISPLAY_SET_CONFIG;
          USBreceiveNumber = false;
          ComTx_VT100_ClearScreen();
          //interface_UpdateUI();
          break;//DISPLAY_SET_PROGRAM_EDIT

        case DISPLAY_SET_PID:        //Edit PID values

          switch(selectPID)
          {
            case DISPLAY_PID_KP:
              pid_Kp_set(inputValue);
              break;//DISPLAY_PID_KP

            case DISPLAY_PID_KI:
              pid_Ki_set(inputValue);
              break;//DISPLAY_PID_KI

            case DISPLAY_PID_KD:
              pid_Kd_set(inputValue);
              break;//DISPLAY_PID_KD
          }
            interface_save_to_flash();
            selectMenu = DISPLAY_SET_CONFIG;
            USBreceiveNumber = false;
            ComTx_VT100_ClearScreen();
          break;//DISPLAY_EDIT_PID

        case DISPLAY_SET_TIME:          //Time set

          switch(selectTime)
          {
            case TIME_HOUR:
                if(inputValue >= 0 && inputValue < 24)
                  htime.Hours = inputValue;
                HAL_RTC_SetTime(&hrtc, &htime, RTC_FORMAT_BIN);
              break;//TIME_HOUR

            case TIME_MINUTE:
                if(inputValue >= 0 && inputValue < 60)
                  htime.Minutes = inputValue;
                HAL_RTC_SetTime(&hrtc, &htime, RTC_FORMAT_BIN);
              break;//TIME_MINUTE

            case TIME_SECOND:
                if(inputValue >= 0 && inputValue < 60)
                  htime.Seconds = inputValue;
                HAL_RTC_SetTime(&hrtc, &htime, RTC_FORMAT_BIN);
                //interface_number_update();
              break;//TIME_SECOND
          }//selectTime

            selectMenu = DISPLAY_SET_CONFIG;
            USBreceiveNumber = false;
            ComTx_VT100_ClearScreen();
          break;//DISPLAY_SET_TIME

        case DISPLAY_SET_DATE:          //Date set

          switch(selectDate)
          {
            case DATE_YEAR:
              if(inputValue >= 0 && inputValue < 100)
                hdate.Year = inputValue;
              HAL_RTC_SetDate(&hrtc, &hdate, RTC_FORMAT_BIN);
              break;//DATE_YEAR

            case DATE_MONTH:
              if(inputValue > 0 && inputValue <= 12)
                hdate.Month = inputValue;
              HAL_RTC_SetDate(&hrtc, &hdate, RTC_FORMAT_BIN);
              break;//DATE_MONTH

            case DATE_DAY:
              if(inputValue > 0 && inputValue <= 31)
                hdate.Date = inputValue;
              HAL_RTC_SetDate(&hrtc, &hdate, RTC_FORMAT_BIN);
              break;//DATE_DAY
            }
              selectMenu = DISPLAY_SET_CONFIG;
              USBreceiveNumber = false;
              ComTx_VT100_ClearScreen();
            break;//DISPLAY_SET_DATE

          case DISPLAY_SET_SAFETY:
            switch(selectSafety)
            {
              case DISPLAY_SAFETY_CURRENT_MAX:
                motor_safety_max_current_set(inputValue);
                break;//DISPLAY_SAFETY_CURRENT_MAX

              case DISPLAY_SAFETY_VOLTAGE_MIN:
                motor_safety_min_voltage_set(inputValue);
                break;//DISPLAY_SAFETY_VOLTAGE_MIN

              case DISPLAY_SAFETY_VOLTAGE_MAX:
                motor_safety_max_voltage_set(inputValue);
                break;//DISPLAY_SAFETY_VOLTAGE_MAX

              case DISPLAY_SAFETY_RAMP_UP_MAX:
                motor_safety_max_ramp_up_set(inputValue);
                break;//DISPLAY_SAFETY_RAMP_UP_MAX

              case DISPLAY_SAFETY_RAMP_DOWN_MAX:
                motor_safety_max_ramp_down_set(inputValue);
                break;//DISPLAY_SAFETY_RAMP_DOWN_MAX

              case DISPLAY_SAFETY_RPM_MAX:
                motor_safety_max_rpm_set(inputValue);
                break;//DISPLAY_SAFETY_RPM_MAX

              case DISPLAY_SAFETY_TEMP1:
              case DISPLAY_SAFETY_TEMP2:
              case DISPLAY_SAFETY_TEMP3:
              case DISPLAY_SAFETY_TEMP4:
              case DISPLAY_SAFETY_TEMP5:
                motor_safety_temp_threshold_set(hdisplay.menu_row - DISPLAY_SAFETY_TEMP1, inputValue);
                break;//TEMP
            }
            interface_save_to_flash();
            selectMenu = DISPLAY_SET_CONFIG;
            USBreceiveNumber = false;
            ComTx_VT100_ClearScreen();
            break;//DISPLAY_SET_SAFETY

/*///////////////////SELECT CONFIG//////////////////////////////*/
/*This section is used for the CONFIG MENU to choode the values to modify
 * Not to be confused with the above section. It is used to accept input values and store them in RAM
 **/
          case DISPLAY_SET_CONFIG: //TODO Config menu options
            switch(selectConfig)
            {
              case DISPLAY_CONFIG_MAIN:
                selectConfig = hdisplay.menu_row;
                ComTx_VT100_ClearScreen();
                hdisplay.menu_row = 0;
                break;//DISPLAY_CONFIG_MAIN

              case DISPLAY_CONFIG_TIME:
                selectMenu = DISPLAY_SET_TIME;
                selectTime = hdisplay.menu_row;
                //hdisplay.menu_row = 0;
                switch(selectTime)
                {
                  case TIME_HOUR:
                    interface_timeHour_set();
                    break;//TIME_HOUR

                  case TIME_MINUTE:
                    interface_timeMinute_set();
                    break;//TIME_MINUTE

                  case TIME_SECOND:
                    interface_timeSecond_set();
                    break;//TIME_SECONDS

                }
                break;//DISPLAY_CONFIG_TIME

              case DISPLAY_CONFIG_DATE:
                selectMenu = DISPLAY_SET_DATE;
                selectDate = hdisplay.menu_row;
                switch(selectDate)
                {
                  case DATE_YEAR:
                    interface_dateYear_set();
                    break;//DATE_YEAR

                  case DATE_MONTH:
                    interface_dateMonth_set();
                    break;//DATE_MONTH

                  case DATE_DAY:
                    interface_dateDay_set();
                    break;//DATE_DAY
                }
                break;//DISPLAY_CONFIG_DATE

              case DISPLAY_CONFIG_PID:
                selectMenu = DISPLAY_SET_PID;
                selectPID = hdisplay.menu_row;
                switch(selectPID)
                {
                  case DISPLAY_PID_KP:
                    interface_kp_set();
                    break;//DISPLAY_PID_KP

                  case DISPLAY_PID_KI:
                    interface_ki_set();
                    break;//DISPLAY_PID_KI

                  case DISPLAY_PID_KD:
                    interface_kd_set();
                    break;//DISPLAY_PID_KD
                }
                break;//DISPLAY_CONFIG_PID

              case DISPLAY_CONFIG_PROGRAM_EDIT:
                selectMenu = DISPLAY_SET_PROGRAM_EDIT;
                selectProgram_display = hdisplay.menu_row;
                switch(selectProgram_display)
                {
                  case DISPLAY_PROGRAM_NUMBER_CYCLES:
                    interface_program_cycleCount_set();
                    break;//DISPLAY_PROGRAM_NUMBER_CYCLES

                  case DISPLAY_PROGRAM_CW_CYCLES:
                    interface_program_cycleCount_CW_set();
                    break;//DISPLAY_PROGRAM_CW_CYCLES

                  case DISPLAY_PROGRAM_CCW_CYCLES:
                    interface_program_cycleCount_CCW_set();
                    break;//DISPLAY_PROGRAM_CCW_CYCLES

                  case DISPLAY_PROGRAM_CYCLE_DURATION:
                    interface_program_revolution_set();
                    break;//DISPLAY_PROGRAM_CYCLE_DURATION

                  case DISPLAY_PROGRAM_CYCLE_TIME_IDLE:
                    interface_program_cycleTime_idle_set();
                    break;//DISPLAY_PROGRAM_CYCLE_TIME_IDLE

                  case DISPLAY_PROGRAM_RAMP_UP:
                    interface_program_ramp_up_set();
                    break;//DISPLAY_PROGRAM_RAMP_UP

                  case DISPLAY_PROGRAM_RAMP_DOWN:
                    interface_program_ramp_down_set();
                    break;//DISPLAY_PROGRAM_RAMP_DOWN
                }
                  break;//DISPLAY_CONFIG_PROGRAM

              case DISPLAY_CONFIG_SAFETY:
                selectMenu = DISPLAY_SET_SAFETY;
                selectSafety = hdisplay.menu_row;
                switch(selectSafety)
                {
                  case DISPLAY_SAFETY_CURRENT_MAX:
                    interface_safety_max_current_set();
                    break;//DISPLAY_SAFETY_CURRENT_MAX

                  case DISPLAY_SAFETY_VOLTAGE_MIN:
                    interface_safety_min_voltage_set();
                    break;//DISPLAY_SAFETY_VOLTAGE_MIN

                  case DISPLAY_SAFETY_VOLTAGE_MAX:
                    interface_safety_max_voltage_set();
                    break;//DISPLAY_SAFETY_VOLTAGE_MAX

                  case DISPLAY_SAFETY_RAMP_UP_MAX:
                    interface_safety_max_ramp_up_set();
                    break;//DISPLAY_SAFETY_RAMP_UP_MAX

                  case DISPLAY_SAFETY_RAMP_DOWN_MAX:
                    interface_safety_max_ramp_down_set();
                    break;//DISPLAY_SAFETY_RAMP_DOWN_MAX

                  case DISPLAY_SAFETY_RPM_MAX:
                    interface_safety_max_rpm_set();
                    break;//DISPLAY_SAFETY_RPM_MAX

                  case DISPLAY_SAFETY_TEMP1:
                  case DISPLAY_SAFETY_TEMP2:
                  case DISPLAY_SAFETY_TEMP3:
                  case DISPLAY_SAFETY_TEMP4:
                  case DISPLAY_SAFETY_TEMP5:
                    interface_safety_temp_set(hdisplay.menu_row - DISPLAY_SAFETY_TEMP1);
                    break;//DISPLAY_SAFETY_TEMPx
                }

                break;//DISPLAY_CONFIG_SAFETY
                //USBreceiveNumber = false;
            }

            break;//DISPLAY_SET_CONFIG
      }
/*USBreceiveNumber informs of an incoming value adjustment , usually of the numeric variety
 * The statement clears the number buffer and resets the index to 0*/
      if(USBreceiveNumber == true)
      {
        //USBreceiveNumber = false;
        memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH); //Clear the number buffer
        clearUSB_receiveBuffer();//clear the receive buffer
        i = 0;
      }
    }
/*    ////////////////////RETURN END*/

    ////////////////////NUMBER BEGIN
    else if ((USBreceiveNumber == true)//TODO Number input
            && (UsbNumber_buffer[USB_NUMBER_BUFFER_LENGTH - 1] == '\0')
            && ((UsbRxData[0] >= '0')
            && (UsbRxData[0] <= '9')
            || (UsbRxData[0] == '-')
            || (UsbRxData[0] == '.')))//allow only 0-9, dot, dash,  and check if array is full
    {
      uint8_t row = UI_ENTER_DATA_ROW;
      uint8_t col = UI_ENTER_DATA_COL;
      if (UsbNumber_buffer[0] == '\0')
        i = 0;

      if ((i == 0) && UsbRxData[0] != '.')//if it's the first character allow to input a '-' (dash)
      {
        UsbNumber_buffer[i] = UsbRxData[0];
        i++;
      }
      else if ((UsbRxData[0] >= '0' && UsbRxData[0] <= '9') || UsbRxData[0] == '.')
      {
        if(UsbRxData[0] == '.')           //is the received character a dot
        {
          for(uint8_t j; j <= i; j++)     //check if there already is a dot
          {
            if (UsbNumber_buffer[j] == '.')
            {
              dot = true;                 //dot detected
              break;
            }
          }
          if (dot == false)               //if there is no dot, print a dot
          {
            UsbNumber_buffer[i] = UsbRxData[0];
            i++;
          }
          dot = false;
        }
        else                              //
        {
          UsbNumber_buffer[i] = UsbRxData[0];
          i++;
        }
      }
      switch(selectMenuMode)
      {
        case DATA_MODE:
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,UsbNumber_buffer);

          break;//DATA_MODE

        case LOG_MODE:

          break;//LOG_MODE

        case CONFIG_MODE:
          switch(selectConfig)
          {
            case DISPLAY_CONFIG_TIME:
                row = UI_TIME_ENTER_DATA_TEXT_ROW;
                col = UI_TIME_ENTER_DATA_TEXT_COL + UI_TIME_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_TIME

            case DISPLAY_CONFIG_DATE:
                row = UI_DATE_ENTER_DATA_TEXT_ROW;
                col = UI_DATE_ENTER_DATA_TEXT_COL + UI_DATE_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_DATE

            case DISPLAY_CONFIG_PID:
                row = UI_PID_ENTER_DATA_TEXT_ROW;
                col = UI_PID_ENTER_DATA_TEXT_COL + UI_PID_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_PID

            case DISPLAY_CONFIG_PROGRAM_EDIT:
                row = UI_PROG_ENTER_DATA_TEXT_ROW;
                col = UI_PROG_ENTER_DATA_TEXT_COL + UI_PROG_ENTER_COL_OFFSET;
              break;

            case DISPLAY_CONFIG_MAIN:
              break;//DISPLAY_CONFIG_MAIN

            case DISPLAY_CONFIG_SAFETY:
                row = UI_SAFETY_ENTER_DATA_TEXT_ROW;
                col = UI_SAFETY_ENTER_DATA_TEXT_COL + UI_SAFETY_ENTER_COL_OFFSET;
              break;//DISPLAY_CONFIG_SAFETY
          }//selectConfig
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,UsbNumber_buffer);
          break;//CONFIG_MODE
      }//selectMenuMode

    }
    ////////////////////NUMBER END

    ////////////////////COMMAND BEGIN
    else      //if char received is not a number nor special character, it's a command
    {
      USB_receiveCommand(UsbRxData); //
    }
    ////////////////////COMMAND END
    USBreceiveFlag = false;
  }
}

void USB_receiveCommand(uint8_t *command) //TODO Button commands from terminal
{
    command[0] = CAPITALIZE_CHAR(command[0]);
    switch(command[0])
    {
      case 'R': //Terminal and UI reset
        if(selectMenuMode == DATA_MODE)
        {
          memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
          //interface_number_update();
          motor_faults_reset();
        }
      break;

      case 'S': //Set reference motor speed
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL)) //Set speed avaliable only in manual mode, when LOG mode is off
        {
          memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
          selectMenu = DISPLAY_SET_MOTOR_SPEED;
          interface_refMotorSpeed_set();
        }
      break;
      case ' ': //Space RUN/STOP command
        if(selectMenuMode == DATA_MODE)
        {
          if(motor_mode_get() == MANUAL)
            motor_status_toggle(); //Motor ON/OFF control//Status variable check in main.c TIM4 Callback funtion
          else if(motor_mode_get() == AUTO)
            motor_program_run_toggle(&hMotor);//Program RUN/STOP//Status variable check in main.c TIM4 Callback
        }
        else if(selectMenuMode == LOG_MODE)
        {
          interface_reset();
          USBlogModeRun ^= true;

          if(USBlogModeRun && (motor_mode_get() == AUTO))
            motor_program_run_set(ON);
          else
            motor_program_run_set(OFF);
          selectMenuMode = LOG_MODE;
          interface_UpdateUI();
        }
      break;

      case 'D': //Direction change
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL))
        {
          if (motor_mode_get() == MANUAL)
            motor_dir_toggle();
        }
      break;

      case 'A': //Auto mode
        if(selectMenuMode == DATA_MODE)
        {
          motor_mode_set(AUTO);
        }
      break;

      case 'M': //Manual mode
        if(selectMenuMode == DATA_MODE)
        {
          motor_program_run_set(OFF);
          motor_status_set(OFF);
          motor_mode_set(MANUAL);
        }
      break;

      case 'P': //Program select
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL))
        {
          memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
          selectMenu = DISPLAY_SET_PROGRAM;
          interface_program_select();
        }
      break;

      case 'L': //LOG mode
        interface_logMode_toggle();
        //selectMenuMode = LOG_MODE;
      break;

      case 'E': //Edit PID values
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL))
        {
          //memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
          //selectMenu = DISPLAY_SET_PID;
          //interface_kp_set();
        }

      break;

      case 'T': //Set Time
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL))
        {
          //memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
          //selectMenu = DISPLAY_SET_TIME;
          //interface_timeHour_set();
        }
        break;

      case 'C': //Reset Cycle count
        if((selectMenuMode == 0) && (motor_mode_get() == MANUAL))
        {
          motor_program_cycle_total_set(0);
        }
        break;

      case '\e': //Special characters
        switch(command[2])
        {
          case '\0': //Escape key, doubles as reset TODO Escape key
            if((USBreceiveNumber == true) && (selectMenuMode == CONFIG_MODE))
            {
              ComTx_VT100_ClearScreen();
              USBreceiveNumber = false;
              selectMenu = DISPLAY_SET_CONFIG;
            }
            else if(selectConfig != DISPLAY_CONFIG_MAIN)
            {
              ComTx_VT100_ClearScreen();
              hdisplay.menu_row = DISPLAY_CONFIG_TIME;
              selectConfig = DISPLAY_CONFIG_MAIN;
            }
            else
            {
              memset(UsbNumber_buffer, 0, USB_NUMBER_BUFFER_LENGTH);
              interface_number_update();
            }
            break;

          case 'A': //Arrow UP
            if(hdisplay.menu_row >  0)
              hdisplay.menu_row--;
            break;

          case 'B': //Arrow DOWN
            if(hdisplay.menu_row < 255)
              hdisplay.menu_row++;
            break;

          case 'C': //Arrow RIGHT
            if(hdisplay.menu_col <  255)
              hdisplay.menu_col++;
            break;

          case 'D': //Arrow LEFT
            if(hdisplay.menu_col > 0)
              hdisplay.menu_col--;
            break;
        }
        break;

      case 'O':
        if((selectMenuMode == DATA_MODE) && (motor_mode_get() == MANUAL))
        {
          interface_config_set();
        }
        break;

      default:

      break;
    }
}

void clearUSB_receiveBuffer(void)
{
  memset(UsbRxData, 0, sizeof(UsbRxData));
}

void interface_reset(void)
{
  selectMenu = DISPLAY_SET_NOTHING;
  selectConfig = DISPLAY_CONFIG_MAIN;
  selectMenuMode = DATA_MODE;
  ComTx_VT100_RestoreDiplayAttr();
  ComTx_VT100_SetCursorHome();
  ComTx_VT100_ClearScreen();
  ComTx_VT100_HideCursor();
}



void interface_number_update(void)
{
  USBreceiveNumber = false;
  interface_reset();
  interface_UpdateUI();
}

void interface_UpdateUI(void)
{
  uint8_t row;
  uint8_t col;
  switch(selectMenuMode)
  {
    case DATA_MODE:////////////////////////////////////////////////////////////////////////////
      USBlogModeRun = 0;

      row = UI_HEADER_ROW;
      col = UI_HEADER_COL;

      ComTx_VT100_RawFull(row,col, VT100_COLOR_ATTR_BRIGHT,
                                     VT100_COLOR_FG_BLACK,
                                     VT100_COLOR_BG_YELLOW,
                                     VT100_MAP_COL,
                                     "DC Motor Controller <C> Rawlplug");
      ComTx_VT100_SetDiplayAttr(VT100_COLOR_ATTR_BRIGHT, VT100_COLOR_FG_WHITE, -1);

      col = 43;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Date       Time");

      row++;
      col = UI_HEADER_COL;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Selected program :"); //enter name of the program (row 2 col 19)

      switch(hMotor.program)
      {
        case MOTOR_PROGRAM_1:
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"CW/CCW Toggle");
          break;
        case MOTOR_PROGRAM_2:
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"START/STOP Single direction");
          break;
        case MOTOR_PROGRAM_3:
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Custom");
          break;
      }

      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Program status   :");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Motor status     :");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Mode             :");
      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------- Electric -------+");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Supply           (V):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Current          (A):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Efficiency       (%):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," PWM duty cycle   (%):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Power            (W):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------------------------+");

      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------ Mechanical ------+");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Ref Speed/Ramp   (rpm):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Ramp up        (rpm/s):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Ramp down      (rpm/s):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Speed            (rpm):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Direction     (CW/CCW):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Cycle         Real/Ref:");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Load               (%):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------------------------+");

      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------- Thermal --------+");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," MOSFET [T1] (C):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Temp2       (C):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Temp3       (C):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Temp4       (C):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Temp5       (C):");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------------------------+");

      row++;

      ComTx_VT100_RawFull(row,col,-1,-1,-1,VT100_MAP_COL,"+--------- PID ----------+");
      /*SAFETY FEATURES DISPLAY*/
      //row = UI_SAFETY_DISPLAY_DATA_TEXT_ROW;
      col = UI_SAFETY_DISPLAY_DATA_TEXT_COL;
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+--------------------------Safety Feature--------------------------+");
      /*SAFETY FEATURES DISPLAY*/

      col = UI_HEADER_COL;
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Kp    :");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Ki    :");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Kd    :");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"+------------------------+");

      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Terminal commands");

      row++;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," |Start/Stop|| DIR ||Auto/Man||Program sel.||Log mode||Config| ");
      ComTx_VT100_RawFull(row++,col, VT100_COLOR_ATTR_BRIGHT,
                                       VT100_COLOR_FG_BLACK,
                                       VT100_COLOR_BG_WHITE,
                                       VT100_MAP_COL,
                                                           " |  SPACE   ||  D  ||   A/M  ||     P      ||    L   ||   O  | ");

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," |Speed||Reset Terminal||Reset Faults|| Reset Cycle Count | ");
      ComTx_VT100_RawFull(row++,col, VT100_COLOR_ATTR_BRIGHT,
                                       VT100_COLOR_FG_BLACK,
                                       VT100_COLOR_BG_WHITE,
                                       VT100_MAP_COL,
                                                           " |  S  ||     Esc      ||     R      ||         C         | ");

      row += 4;

      ComTx_VT100_RestoreDiplayAttr();
      ComTx_VT100_RawFull(row++,col,VT100_COLOR_ATTR_BRIGHT,
          VT100_COLOR_FG_BLACK,
          VT100_COLOR_BG_YELLOW,
          VT100_MAP_COL,
          "  " "Copyright <C> Rawlplug" "  ");
      break;

    case LOG_MODE://TODO Log Mode Text
      if(USBlogModeRun == 0)
      {
        ComTx_VT100_RawFull(1,1,-1,-1,-1,VT100_MAP_COL,"Enable LOG in terminal and press 'SPACE' to continue, or press 'ESC' to exit.\r\n");
      }
      else
      {
      row = 1;
      col = 1;
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Date Time Voltage[V] Current[A] PWMduty[%] Power[W] RefSpeed_Ramp[RPM] RealSpeed[RPM] "\
          "Dir[CW/CCW] Cycle_Nr Ref_Cycle Temp1[C] Temp2[C] Temp3[C] Temp4[C] Temp5[C]\r\n");
      }

      break;

    case CONFIG_MODE:

      break;
  }
}

void interface_displayData(displayData_t *hData)
{
  interface_updateData(hData);
  //HAL_RTC_GetTime(&hrtc, &htime, RTC_FORMAT_BIN);
  //HAL_RTC_GetDate(&hrtc, &hdate, RTC_FORMAT_BIN);
  uint8_t row;
  uint8_t col;
  switch(selectMenuMode)
  {
    case DATA_MODE:////////////////////////////////////////////////////////////////////////////
      /*DISPLAY DATA*/

      row = 2;
      col = 43;
      ComTx_VT100_PrintfFull(row,col,-1,-1,-1,VT100_MAP_COL,"%d/%d/%d  ", hdate.Date, hdate.Month, hdate.Year + 2000);
      ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d:%d:%d", htime.Hours, htime.Minutes, htime.Seconds);


      row = /*UI_DATA_ROW*/3;
      col = /*UI_DATA_COL*/21;

      ComTx_VT100_PrintfFull(row++,col,-1,-1,(hData->motor->program == 0 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," 1 ");
      ComTx_VT100_PrintfFull(-1,-1,-1,-1,(hData->motor->program == 1 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," 2 ");
      ComTx_VT100_PrintfFull(-1,-1,-1,-1,(hData->motor->program == 2 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," 3 ");

      row += 2;
      //row++;
      col = /*UI_DATA_COL*/21;

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL, hData->motor->program_run ? "RUNNING" : "STOP");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL, hData->motor->status ? "RUNNING" : "STOP");
      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL, hData->motor->mode   ? "AUTO" : "MANUAL");


      row += 2;/*UI_DATA_ROW 10*/;
      col = /*UI_DATA_COL*/25;

      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->motor->voltage);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->motor->current);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->motor->efficiency);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", (hData->motor->pwm_duty >= 0) ? hData->motor->pwm_duty : -hData->motor->pwm_duty);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->motor->power);

      row += 3;
      //row = /*UI_DATA_ROW*/18;
      col = /*UI_DATA_COL*/27;

      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f/%.2f", hData->motor->ref_speed, hData->motor->ref_speed_ramp);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->program->ramp_up);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->program->ramp_down);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", hData->motor->speed);

      ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL, hData->motor->direction ? "CCW" : "CW");

      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%d/%d",motor_program_cycle_total_get(), motor_program_cycle_ref_total_get());
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%d", hData->motor->load);

      row += 3;
      //row = /*UI_DATA_ROW*/26;
      col = /*UI_DATA_COL*/20;

      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->temp_c.temperature_celcious[0]);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->temp_c.temperature_celcious[1]);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->temp_c.temperature_celcious[2]);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->temp_c.temperature_celcious[3]);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->temp_c.temperature_celcious[4]);

      row +=3;
      /*SAFETY DATA DISPLAY BEGIN*/
      //row = UI_SAFETY_DISPLAY_DATA_TEXT_ROW + 1 ;
      col = UI_SAFETY_DISPLAY_DATA_TEXT_COL;
      ComTx_VT100_RawFull(row,col,-1,-1,motor_fault_OV_protection_get() ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OVP |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_UV_protection_get() ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| UVP |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OC_protection_get() ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OCP |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OT_protection_get(0) ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OT1 |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OT_protection_get(1) ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OT2 |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OT_protection_get(2) ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OT3 |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OT_protection_get(3) ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OT4 |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_OT_protection_get(4) ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| OT5 |");
      ComTx_VT100_RawFull(-1,-1,-1,-1,motor_fault_encoder_fail_get() ? VT100_COLOR_BG_RED: VT100_COLOR_BG_GREEN,VT100_MAP_NONE,"| ENC FAIL |");

      /*SAFETY DATA DISPLAY END*/

      //row--;
      //row = /*UI_DATA_ROW*/34;
      col = /*UI_DATA_COL*/11;

      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_NONE,"%.4f", hData->motor->pid_regulator.Kp);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_NONE,"%.4f", hData->motor->pid_regulator.Ki);
      ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_NONE,"%.4f", hData->motor->pid_regulator.Kd);
      //ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.2f", data.pid_error);
      break;

    case LOG_MODE://TODO Log Mode data
      if((USBlogModeRun == true) && (motor_program_run_get() == ON))
      {
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d/%d/%d ", hdate.Date, hdate.Month, hdate.Year + 2000);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d:%d:%d ", htime.Hours, htime.Minutes, htime.Seconds);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.1f ", hData->motor->voltage);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->motor->current);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", (hData->motor->pwm_duty >= 0) ? hData->motor->pwm_duty : -hData->motor->pwm_duty);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d ", hData->motor->power);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->motor->ref_speed_ramp);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->motor->speed);
        ComTx_VT100_RawFull(-1,-1,-1,-1,-1,VT100_MAP_COL, hData->motor->direction ? "CCW " : "CW ");
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d ", hData->program->cycle_total);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%d ", hData->program->cycle_ref_total);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->temp_c.temperature_celcious[0]);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->temp_c.temperature_celcious[1]);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->temp_c.temperature_celcious[2]);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->temp_c.temperature_celcious[3]);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"%.2f ", hData->temp_c.temperature_celcious[4]);
        ComTx_VT100_PrintfFull(-1,-1,-1,-1,-1,VT100_MAP_COL,"\r\n");
      }
      break;

    case CONFIG_MODE: // TODO Config mode

      switch(selectConfig)
      {
        case DISPLAY_CONFIG_MAIN:
            row = 2;
            col = 1;
            ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG MENU ");

            row++;

            ComTx_VT100_RawFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_CONFIG_TIME ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," CONFIG TIME ");
            ComTx_VT100_RawFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_CONFIG_DATE ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," CONFIG DATE ");
            ComTx_VT100_RawFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_CONFIG_PID ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," CONFIG PID ");
            ComTx_VT100_RawFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_CONFIG_PROGRAM_EDIT ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," CONF.MOT.PROG. ");
            ComTx_VT100_RawFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_CONFIG_SAFETY ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL," CONFIG SAFETY ");
            row = row + 3;

            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to enter sub-menu");
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

            /*Prevent menu from overshooting*/
            if (hdisplay.menu_row > DISPLAY_CONFIG_MAIN - 1)
              hdisplay.menu_row = DISPLAY_CONFIG_MAIN - 1;
          break;//DISPLAY_CONFIG_MAIN

        case DISPLAY_CONFIG_TIME:
            row = 2;
            col = 1;
            ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG TIME ");

            row++;

            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == TIME_HOUR ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"HOUR    : %d", htime.Hours);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == TIME_MINUTE ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MINUTE  : %d", htime.Minutes);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == TIME_SECOND ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"SECOND  : %d", htime.Seconds);

            row = row + 3;
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to edit value");
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

            /*Prevent menu from overshooting*/
            if (hdisplay.menu_row > TIME_SECOND)
              hdisplay.menu_row = TIME_SECOND;
          break;//DISPLAY_CONFIG_TIME

        case DISPLAY_CONFIG_DATE:
            row = 2;
            col = 2;
            ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG DATE ");

            row++;

            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DATE_YEAR ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"YEAR    : %d", hdate.Year);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DATE_MONTH ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MONTH   : %d", hdate.Month);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DATE_DAY ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"DAY     : %d", hdate.Date);

            row = row + 3;
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to edit value");
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

            /*Prevent menu from overshooting*/
            if (hdisplay.menu_row > DATE_DAY)
              hdisplay.menu_row = DATE_DAY;
          break;//DISPLAY_CONFIG_DATE

        case DISPLAY_CONFIG_PID:
            row = 2;
            col = 2;
            ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG PID ");

            row++;

            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PID_KP ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"Kp      : %.4f", hData->motor->pid_regulator.Kp);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PID_KI ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"Ki      : %.4f", hData->motor->pid_regulator.Ki);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PID_KD ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"Kd      : %.4f", hData->motor->pid_regulator.Kd);

            row = row + 3;
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to edit value");
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

            /*Prevent menu from overshooting*/
            if (hdisplay.menu_row > DISPLAY_PID_KD)
              hdisplay.menu_row = DISPLAY_PID_KD;
          break;//DISPLAY_CONFIG_PID

        case DISPLAY_CONFIG_PROGRAM_EDIT:
            row = 2;
            col = 2;
            ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG PROGRAM ");

            row++;

            ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"Total cycle: %d (N Cycle * (CW Cycle + CCW Cycle)", motor_program_cycle_ref_total_get());

            row++;

            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_NUMBER_CYCLES ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"N   Cycle: %d",motor_program_cycleCount_get());
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_CW_CYCLES ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"CW  Cycle: %d",motor_program_ref_cycleCount_CW_get());
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_CCW_CYCLES ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"CCW Cycle: %d",motor_program_ref_cycleCount_CCW_get());
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_CYCLE_DURATION ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"Rev per cycle: %d",hData->program->cycle_revolutions);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_CYCLE_TIME_IDLE ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"Idle Time [ms]: %d",hData->program->cycle_time_idle);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_RAMP_UP ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"RAMP UP  : %.3f",hData->program->ramp_up);
            ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_PROGRAM_RAMP_DOWN ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"RAMP DOWN: %.3f",hData->program->ramp_down);

            row = row + 3;
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to edit value");
            ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

            /*Prevent menu from overshooting*/
            if (hdisplay.menu_row > DISPLAY_PROGRAM_RAMP_DOWN)
              hdisplay.menu_row = DISPLAY_PROGRAM_RAMP_DOWN;
          break;//DISPLAY_CONFIG_PROGRAM_EDIT

        case DISPLAY_CONFIG_SAFETY:
          row = 2;
          col = 2;
          ComTx_VT100_RawFull(row++,col,-1,-1,VT100_COLOR_BG_CYAN,VT100_MAP_COL," CONFIG SAFETY ");

          row++;

          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_CURRENT_MAX ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Current[A]: %.2f",hData->safety->safety_max_current);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_VOLTAGE_MAX ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Voltage[V]: %.2f",hData->safety->safety_max_voltage);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_VOLTAGE_MIN ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MIN Voltage[V]: %.2f",hData->safety->safety_min_voltage);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_RAMP_UP_MAX ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Ramp up[rpm/s]: %.2f",hData->safety->safety_max_ramp_up);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_RAMP_DOWN_MAX ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Ramp down[rpm/s]: %.2f",hData->safety->safety_max_ramp_down);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_RPM_MAX ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX RPM  : %.2f",hData->safety->safety_max_rpm);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_TEMP1 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Temp1: %.2f",hData->safety->safety_temp_threshold[0]);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_TEMP2 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Temp2: %.2f",hData->safety->safety_temp_threshold[1]);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_TEMP3 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Temp3: %.2f",hData->safety->safety_temp_threshold[2]);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_TEMP4 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Temp4: %.2f",hData->safety->safety_temp_threshold[3]);
          ComTx_VT100_PrintfFull(row++,col,-1,-1,(hdisplay.menu_row == DISPLAY_SAFETY_TEMP5 ? VT100_COLOR_BG_WHITE : -1),VT100_MAP_COL,"MAX Temp5: %.2f",hData->safety->safety_temp_threshold[4]);

          row = row + 3;
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ENTER' to edit value");
          ComTx_VT100_RawFull(row++,col,-1,-1,-1,VT100_MAP_COL," Press 'ESC' to return to previous menu");

          /*Prevent menu from overshooting*/
          if (hdisplay.menu_row > DISPLAY_SAFETY_TEMP5)
            hdisplay.menu_row = DISPLAY_SAFETY_TEMP5;
          break;//DISPLAY_CONFIG_SAFETY
      }
      break;//CONFIG_MODE
  }
}

void interface_updateData(displayData_t *hData)
{
  HAL_RTC_GetTime(&hrtc, &htime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &hdate, RTC_FORMAT_BIN);
  /*Get data from motor struct*/
  hData->motor->status = motor_status_get();
  hData->motor->mode = motor_mode_get();
  hData->motor->ref_speed = motor_refSpeed_get();
  hData->motor->speed = motor_speed_get();
  hData->motor->direction = motor_dir_get();
  hData->motor->pwm_duty = motor_pwmDutyCycle_get();
  //hData->motor->current = motor_current_get();
  //hData->motor->voltage = motor_voltage_get();
  hData->motor->torque = motor_torque_get();
  hData->motor->load = motor_load_get();
  hData->motor->power = motor_power_get();
  //hData->motor->efficiency = motor_efficiency_get();
  //hData->motor->pid_regulator.Kp = pid_Kp_get();
  //hData->motor->pid_regulator.Ki = pid_Ki_get();
  //hData->motor->pid_regulator.Kd = pid_Kd_get();
  /*****************************/
}



//Serial terminal interface, code goes here TODO
