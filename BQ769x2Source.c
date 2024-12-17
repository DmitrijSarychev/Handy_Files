/*
 * BQ769x2Source.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*//* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (Non-USER sections generated from STM32CubeMX software)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
//  BQ76952EVM demo code for STM32 NUCLEO-F103RB + BQ76952EVM
//
//  Connection description: The I2C SCL and SDA pins are the only pin connections required between the
//  NUCLEO board and the BQ76952EVM for this demo code. Also a ground connection should be made between the 2 boards.
//  The ALERT, RST_SHUT, and DFETOFF pins are also configured on the MCU and can be used as shown.
//
/*                                     /|\ /|\*/
//                   STM32             5k   |
//                 -----------------    |  5k
//                |             PB8 |---+---|-- I2C Clock (SCL)
//                |                 |       |
//                |             PB9 |---------+-- I2C Data (SDA)
//                |                 |
//     DFETOFF ---| PA8             |
//                |                 |
//   RST_SHUT  ---| PA9             |--- Green LED
//                |                 |
//      ALERT  ---| PA10            |
//                |                 |

#include <main.h>
#include <gpio.h>
#include <stdio.h>
#include "stdint.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
//#include "usb_device.h"
#include "terminal.h"
#include "interface.h"
#include "BQ769x2Header.h"

BQ769x2_BMS_t hbms;

void BQ769x2_bms_process(void)
{
  BQ769x2_CB_ActiveCells();
  hbms.AlarmBits = BQ769x2_ReadAlarmStatus();
  hbms.AlarmRawBits = BQ769x2_ReadAlarmRawStatus();
  hbms.Alarm_Enable = BQ769x2_ReadAlarmEnable();
  //if (hbms.AlarmBits & 0x80) //Check if FULLSCAN is complete. If set, new measurements are available
  //{
        BQ769x2_ReadAllVoltages();
        uint16_t bms_current = (uint16_t)BQ769x2_ReadCurrent() * 10;
        if(bms_current & 0x8000)
        {
          hbms.Pack_Current = 0xffff0000 | bms_current;
        }
        else
        {
          hbms.Pack_Current = bms_current;
        }

        hbms.Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
        hbms.Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
    //DirectCommands(AlarmStatus, 0x0080, W);  // Clear the FULLSCAN bit
  //}

  if (hbms.AlarmBits & 0xC000)
  {  // If Safety Status bits are showing in AlarmStatus register
    BQ769x2_ReadSafetyStatus(); // Read the Safety Status registers to find which protections have triggered
    if (hbms.ProtectionsTriggered & 1)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); }// Turn on the LED to indicate Protection has triggered
      DirectCommands(AlarmStatus, 0xF800, W); // Clear the Safety Status Alarm bits.
    }
  else
  {
    if (hbms.ProtectionsTriggered & 1) {
      BQ769x2_ReadSafetyStatus();
      if (!(hbms.ProtectionsTriggered & 1))
      {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      }
    } // Turn off the LED if Safety Status has cleared which means the protection condition is no longer present
  }
  //BQ769x2_ReadPFStatus();
  BQ769x2_ReadFETStatus();
}

void rst_shut_pin_low_set(void)
{
    HAL_GPIO_WritePin(RST_SHUT_GPIO_Port, RST_SHUT_Pin, GPIO_PIN_RESET);  // RST_SHUT pin set low
}
void dfet_off_pin_low_set(void)
{
    HAL_GPIO_WritePin(DFET_OFF_GPIO_Port, DFET_OFF_Pin, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
}

void delayUS(uint32_t us)
{   // Sets the delay in microseconds.
  __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
  unsigned char i;
  unsigned char checksum = 0;

  for(i=0; i<len; i++)
    checksum += ptr[i];

  checksum = 0xff & ~checksum;

  return(checksum);
}

unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
  unsigned char i;
  unsigned char crc=0;
  while(len--!=0)
  {
    for(i=0x80; i!=0; i/=2)
    {
      if((crc & 0x80) != 0)
      {
        crc *= 2;
        crc ^= 0x107;
      }
      else
        crc *= 2;

      if((*ptr & i)!=0)
        crc ^= 0x107;
    }
    ptr++;
  }
  return(crc);
}

void I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
#if CRC_Mode

    uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    uint8_t crc_count = 0;
    crc_count = count * 2;
    uint8_t crc1stByteBuffer [3] = {0x10, reg_addr, reg_data[0]};
    unsigned int j;
    unsigned int i;
    uint8_t temp_crc_buffer [3];

    TX_Buffer[0] = reg_data[0];
    TX_Buffer[1] = CRC8(crc1stByteBuffer,3);

    j = 2;
    for(i=1; i<count; i++)
    {
      TX_Buffer[j] = reg_data[i];
      j = j + 1;
      temp_crc_buffer[0] = reg_data[i];
      TX_Buffer[j] = CRC8(temp_crc_buffer,1);
      j = j + 1;
    }
    HAL_I2C_Mem_Write(I2C_BMS, DEV_ADDR, reg_addr, 1, TX_Buffer, crc_count, 1000);
#else
  //HAL_I2C_Master_Transmit(I2C_BMS, DEV_ADDR, data_buffer, count, 1000);
  HAL_I2C_Mem_Write(I2C_BMS, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
}

int I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{

#if CRC_Mode
    unsigned int RX_CRC_Fail = 0;  // reset to 0. If in CRC Mode and CRC fails, this will be incremented.
    uint8_t RX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    uint8_t crc_count = 0;
    uint8_t ReceiveBuffer [10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    crc_count = count * 2;
    unsigned int j;
    unsigned int i;
    unsigned char CRCc = 0;
    uint8_t temp_crc_buffer [3];

    HAL_I2C_Mem_Read(I2C_BMS, DEV_ADDR, reg_addr, 1, ReceiveBuffer, crc_count, 1000);
    uint8_t crc1stByteBuffer [4] = {0x10, reg_addr, 0x11, ReceiveBuffer[0]};
    CRCc = CRC8(crc1stByteBuffer,4);
    if (CRCc != ReceiveBuffer[1])
    {
      RX_CRC_Fail += 1;
    }
    RX_Buffer[0] = ReceiveBuffer[0];

    j = 2;
    for (i=1; i<count; i++)
    {
      RX_Buffer[i] = ReceiveBuffer[j];
      temp_crc_buffer[0] = ReceiveBuffer[j];
      j = j + 1;
      CRCc = CRC8(temp_crc_buffer,1);
      if (CRCc != ReceiveBuffer[j])
        RX_CRC_Fail += 1;
      j = j + 1;
    }
    CopyArray(RX_Buffer, reg_data, crc_count);
#else
  HAL_I2C_Mem_Read(I2C_BMS, DEV_ADDR, reg_addr, 1, reg_data, count, 1000);
#endif
  return 0;
}

void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
  uint8_t TX_Buffer[2] = {0x00, 0x00};
  uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  //TX_RegData in little endian format
  TX_RegData[0] = reg_addr & 0xff;
  TX_RegData[1] = (reg_addr >> 8) & 0xff;
  TX_RegData[2] = reg_data & 0xff; //1st byte of data

  switch(datalen)
    {
    case 1: //1 byte datalength
          I2C_WriteReg(0x3E, TX_RegData, 3);
      HAL_Delay(20);
      TX_Buffer[0] = Checksum(TX_RegData, 3);
      TX_Buffer[1] = 0x05; //combined length of register address and data
          I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      HAL_Delay(20);
      break;
    case 2: //2 byte datalength
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      I2C_WriteReg(0x3E, TX_RegData, 4);
      HAL_Delay(20);
      TX_Buffer[0] = Checksum(TX_RegData, 4);
      TX_Buffer[1] = 0x06; //combined length of register address and data
          I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      HAL_Delay(20);
      break;
    case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
      TX_RegData[3] = (reg_data >> 8) & 0xff;
      TX_RegData[4] = (reg_data >> 16) & 0xff;
      TX_RegData[5] = (reg_data >> 24) & 0xff;
      I2C_WriteReg(0x3E, TX_RegData, 6);
      HAL_Delay(20);
      TX_Buffer[0] = Checksum(TX_RegData, 6);
      TX_Buffer[1] = 0x08; //combined length of register address and data
          I2C_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
      HAL_Delay(20);
      break;
    }
}

void CommandSubcommands(uint16_t command) //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
{ //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

  uint8_t TX_Reg[2] = {0x00, 0x00};

  //TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;

  I2C_WriteReg(0x3E,TX_Reg,2);
  HAL_Delay(20);
}

void Subcommands(uint16_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
{
  //security keys and Manu_data writes dont work with this function (reading these commands works)
  //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
  uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t TX_Buffer[2] = {0x00, 0x00};

  //TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;

  if (type == R) {//read
    I2C_WriteReg(0x3E,TX_Reg,2);
    HAL_Delay(20);
    I2C_ReadReg(0x40, hbms.RX_32Byte, 32); //RX_32Byte is a global variable
  }
  else if (type == W) {
    //FET_Control, REG12_Control
    TX_Reg[2] = data & 0xff;
    I2C_WriteReg(0x3E,TX_Reg,3);
    //HAL_Delay(10);
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05; //combined length of registers address and data
    I2C_WriteReg(0x60, TX_Buffer, 2);
    //HAL_Delay(10);
  }
  else if (type == W2){ //write data with 2 bytes
    //CB_Active_Cells, CB_SET_LVL
    TX_Reg[2] = data & 0xff;
    TX_Reg[3] = (data >> 8) & 0xff;
    I2C_WriteReg(0x3E,TX_Reg,4);
    //HAL_Delay(10);
    TX_Buffer[0] = Checksum(TX_Reg, 4);
    TX_Buffer[1] = 0x06; //combined length of registers address and data
    I2C_WriteReg(0x60, TX_Buffer, 2);
    //HAL_Delay(10);
  }
}

void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{ //type: R = read, W = write
  uint8_t TX_data[2] = {0x00, 0x00};

  //little endian format
  TX_data[0] = data & 0xff;
  TX_data[1] = (data >> 8) & 0xff;

  if (type == R)
  {//Read
    I2C_ReadReg(command, hbms.RX_data, 2); //RX_data is a global variable
    //HAL_Delay(20);
  }
  if (type == W)
  {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
    I2C_WriteReg(command,TX_data,2);
    //HAL_Delay(20);
  }
}

void BQ769x2_Init(void)
{
#if DEBUG_ENABLE
  uint16_t row = 1;
  uint16_t col = 1;
#endif
  // Configures all parameters in device RAM

  // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
  // See TRM for full description of CONFIG_UPDATE mode
  CommandSubcommands(SET_CFGUPDATE);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s] SET_CFGUPDATE",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  //ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"%.1f", hData->motor->voltage);

  // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
  // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
  // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
  // a full description of the register and the bits will pop up on the screen.

  // 'CC Gain' - 0x91A8 = 1.04
  // Setting current calibration value to match SHUNT resistor
  BQ769x2_SetRegister(CCGain, 1.040f, 4);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CC Gain]:1.04",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // 'Power Config' - 0x9234 = 0x2D80
  // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
  // Set wake speed bits to 00 for best performance
  BQ769x2_SetRegister(PowerConfig, 0x2982, 2);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][PowerConfig]:0x2D80",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
  BQ769x2_SetRegister(REG0Config, 0x01, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][REG0Config]:0x01",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
  BQ769x2_SetRegister(REG12Config, 0x9D, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][REG12Config]:0x0D (3.3V)",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set CFETOFF pin to control CHG FET - 0x92FA = 0x02 (set to 0x00 to disable)
  BQ769x2_SetRegister(CFETOFFPinConfig, 0x02, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CFETOFFPinConfig]:0x02",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable) Only DSG FET - 0x92FB = 0x02
  BQ769x2_SetRegister(DFETOFFPinConfig, 0x02, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][DFETOFFPinConfig]:0x02",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up ALERT Pin - 0x92FC = 0x2A
  // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
  // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
  BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][ALERTPinConfig]:0x2A",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
  BQ769x2_SetRegister(TS1Config, 0x07, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][TS1Config]:0x07",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
  BQ769x2_SetRegister(TS3Config, 0x0F, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][TS3Config]:0x0F",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
  BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][HDQPinConfig]:0x00",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set DA Configuration - 0x9303 = 0x06 set current to read in 10mA (default 0x05 set current to read in 1mA)
  BQ769x2_SetRegister(DAConfiguration, 0x06, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][DAConfiguration]:0x05",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
  BQ769x2_SetRegister(VCellMode, 0x020F, 2);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][VCellMode]:0x020F (5 Cell)",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
  // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
  // COV (over-voltage), CUV (under-voltage)
  BQ769x2_SetRegister(EnabledProtectionsA, 0xFC, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][EnabledProtectionsA]:0xBC",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
  // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
  // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
  BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][EnabledProtectionsB]:0xF7",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xFEFF
  BQ769x2_SetRegister(DefaultAlarmMask, 0xF81C, 2);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][DefaultAlarmMask]:0xF882",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
  // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
  BQ769x2_SetRegister(BalancingConfiguration, 0x0A, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][BalancingConfiguration]:0x0F",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  /*
   * Set balancing interval 0x9339 = 20 s. The cell balancing algorithm recalculates which cell to balance after this many seconds. Once a
   * cell is chosen, balancing on that cell will continue for this interval unless one of the conditions which blocks
   * */
    BQ769x2_SetRegister(CellBalanceInterval, 20, 1);
  #if DEBUG_ENABLE
    ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceInterval]:0x01",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
  #endif

  //Set max cells for balancing 0x933B = 0x01 - number of cells for simultaneous balancing
  BQ769x2_SetRegister(CellBalanceMaxCells, 0x01, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceMaxCells]:0x01",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  //Set balance minimum charge voltage 0x933B = 0xC1C (3100mV)
  BQ769x2_SetRegister(CellBalanceMinCellVCharge, 3100, 2);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceMinCellVCharge]:3100mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  //Set balance minimum charge voltage delta 0x933D = 0x28 (40mV)
  BQ769x2_SetRegister(CellBalanceMinDeltaCharge, 40, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceMinDeltaCharge]:40mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  //Set balance minimum charge voltage delta stop (histeresis) 0x933E = 0x28 (20mV)
  BQ769x2_SetRegister(CellBalanceStopDeltaCharge, 20, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceStopDeltaCharge]:20mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  //Set balance minimum relax voltage 0x933F = 0xC1C (3100mV)
  BQ769x2_SetRegister(CellBalanceMinCellVRelax, 3100, 2);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceMinCellVRelax]:3100mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  //Set balance minimum relax voltage delta 0x933D = 0x28 (40mV)
  BQ769x2_SetRegister(CellBalanceMinDeltaRelax, 40, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceMinDeltaRelax]:40mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  //Set balance minimum relax voltage delta stop (histeresis) 0x933E = 0x28 (20mV)
  BQ769x2_SetRegister(CellBalanceStopDeltaRelax, 20, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CellBalanceStopDeltaRelax]:20mV",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif

  // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
  // CUV Threshold is this value multiplied by 50.6mV
  BQ769x2_SetRegister(CUVThreshold, 0x31, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][CUVThreshold]:0x31",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
  // COV Threshold is this value multiplied by 50.6mV
  BQ769x2_SetRegister(COVThreshold, 0x55, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][COVThreshold]:0x55",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
  BQ769x2_SetRegister(OCCThreshold, 0x04, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][OCCThreshold]:0x05",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
  BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][OCD1Threshold]:0x0A",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  units of 20mV
  BQ769x2_SetRegister(SCDThreshold, 0x05, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][SCDThreshold]:0x05",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 µs; min value of 1
  BQ769x2_SetRegister(SCDDelay, 0x03, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][SCDDelay]:0x03",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
  // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
  BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s][SCDLLatchLimit]:0x01",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
  // Exit CONFIGUPDATE mode  - Subcommand 0x0092
  CommandSubcommands(EXIT_CFGUPDATE);
#if DEBUG_ENABLE
  ComTx_VT100_PrintfFull(row++,col,-1,-1,-1,VT100_MAP_COL,"[%d][%s][%s] EXIT_CFGUPDATE",__LINE__,DbgTraceGetFileName(__FILE__),__FUNCTION__);
#endif
}

void BQ769x2_Reset(void)
{

}

void main_BQ769x2_Init(void)
{
  rst_shut_pin_low_set();
  dfet_off_pin_low_set();
  HAL_Delay(100);

  CommandSubcommands(BQ769x2_RESET); //Resets the BQ769x2 registers
  HAL_Delay(600);
  BQ769x2_Init();       //Configure all of the BQ769x2 register settings
  //DirectCommands(AlarmEnable, 0x789D, W);   //Alarm mask to enable Alarm and ALERT Pin (0xFEFF default)
  HAL_Delay(100);
  CommandSubcommands(FET_ENABLE);    //Enable the CHG and DSG FETs
  HAL_Delay(100);
  CommandSubcommands(SLEEP_ENABLE); //Sleep mode is enabled by default. For this example, Sleep is disabled to
                       //demonstrate full-speed measurements in Normal mode.
  HAL_Delay(240);
  //delayUS(2400000);  //wait to start measurements after FETs close
  interface_reset();
}
//  ********************************* FET Control Commands  ***************************************

void BQ769x2_BOTHOFF(void)
{
  // Disables all FETs using the DFETOFF (BOTHOFF) pin
  // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
  HAL_GPIO_WritePin(DFET_OFF_GPIO_Port, DFET_OFF_Pin, GPIO_PIN_SET);  // DFETOFF pin (BOTHOFF) set high
}

void BQ769x2_RESET_BOTHOFF(void)
{
  // Resets DFETOFF (BOTHOFF) pin
  // The DFETOFF pin on the BQ76952EVM should be connected to the MCU board to use this function
  HAL_GPIO_WritePin(DFET_OFF_GPIO_Port, DFET_OFF_Pin, GPIO_PIN_RESET);  // DFETOFF pin (BOTHOFF) set low
}

void BQ769x2_ReadFETStatus(void)
{
  // Read FET Status to see which FETs are enabled
  DirectCommands(FETStatus, 0x00, R);
  hbms.FET_Status = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  hbms.DSG = ((0x4 & hbms.RX_data[0])>>2);// discharge FET state
  hbms.CHG = (0x1 & hbms.RX_data[0]);// charge FET state
  hbms.PCHG = ((0x2 & hbms.RX_data[0])>>1);// pre-charge FET state
  hbms.PDSG = ((0x8 & hbms.RX_data[0])>>3);// pre-discharge FET state
}

void BQ769x2_CB_ActiveCells(void)
{
  //DirectCommands(CB_ACTIVE_CELLS, 0x0F, W);
/*  Subcommands(CB_SET_LVL, 3500, W);
  Subcommands(CB_SET_LVL, 3500, R);*/

/*  DirectCommands(CB_SET_LVL, 0x00, R);*/
  //hbms.CB_SetLVL = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  Subcommands(CB_ACTIVE_CELLS, 0x00, R);
  hbms.CB_ActiveCells = (hbms.RX_32Byte[1] << 8) + hbms.RX_32Byte[0];
  //delayUS(1000000);


}

// ********************************* End of FET Control Commands *********************************

// ********************************* BQ769x2 Power Commands   *****************************************

void BQ769x2_ShutdownPin(void)
{
  // Puts the device into SHUTDOWN mode using the RST_SHUT pin
  // The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // Sets RST_SHUT pin
}

void BQ769x2_ReleaseShutdownPin(void)
{
  // Releases the RST_SHUT pin
  // The RST_SHUT pin on the BQ76952EVM should be connected to the MCU board to use this function
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // Resets RST_SHUT pin
}

// ********************************* End of BQ769x2 Power Commands   *****************************************


// ********************************* BQ769x2 Status and Fault Commands   *****************************************

uint16_t BQ769x2_ReadAlarmStatus(void)
{
  // Read this register to find out why the ALERT pin was asserted
  DirectCommands(AlarmStatus, 0x00, R);
  return (hbms.RX_data[1]*256 + hbms.RX_data[0]);
}

uint16_t BQ769x2_ReadAlarmRawStatus(void)
{
  // Read this register to find out why the ALERT pin was asserted
  DirectCommands(AlarmRawStatus, 0x00, R);
  return (hbms.RX_data[1]*256 + hbms.RX_data[0]);
}

uint16_t BQ769x2_ReadAlarmEnable(void)
{
  // Read this register to find out why the ALERT pin was asserted
  DirectCommands(AlarmEnable, 0x00, R);
  return (hbms.RX_data[1]*256 + hbms.RX_data[0]);
}

void BQ769x2_ReadSafetyStatus(void)
{ //good example functions
  // Read Safety Status A/B/C and find which bits are set
  // This shows which primary protections have been triggered
  DirectCommands(SafetyStatusA, 0x00, R);
  hbms.value_SafetyStatusA = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  //Example Fault Flags
  hbms.UV_Fault = ((0x4 & hbms.RX_data[0])>>2);
  hbms.OV_Fault = ((0x8 & hbms.RX_data[0])>>3);
  hbms.SCD_Fault = ((0x8 & hbms.RX_data[1])>>3);
  hbms.OCD_Fault = ((0x2 & hbms.RX_data[1])>>1);
  DirectCommands(SafetyStatusB, 0x00, R);
  hbms.value_SafetyStatusB = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  DirectCommands(SafetyStatusC, 0x00, R);
  hbms.value_SafetyStatusC = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  if ((hbms.value_SafetyStatusA + hbms.value_SafetyStatusB + hbms.value_SafetyStatusC) > 1) {
    hbms.ProtectionsTriggered = 1; }
  else {
    hbms.ProtectionsTriggered = 0; }
}

void BQ769x2_ReadPFStatus(void)
{
  // Read Permanent Fail Status A/B/C and find which bits are set
  // This shows which permanent failures have been triggered
  DirectCommands(PFStatusA, 0x00, R);
  hbms.value_PFStatusA = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  DirectCommands(PFStatusB, 0x00, R);
  hbms.value_PFStatusB = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
  DirectCommands(PFStatusC, 0x00, R);
  hbms.value_PFStatusC = (hbms.RX_data[1]*256 + hbms.RX_data[0]);
}

// ********************************* End of BQ769x2 Status and Fault Commands   *****************************************


// ********************************* BQ769x2 Measurement Commands   *****************************************


uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
  //RX_data is global var
  DirectCommands(command, 0x00, R);
  if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
    return (hbms.RX_data[1]*256 + hbms.RX_data[0]); //voltage is reported in mV
  }
  else {//stack, Pack, LD
    return 10 * (hbms.RX_data[1]*256 + hbms.RX_data[0]); //voltage is reported in 0.01V units
  }

}
void BQ769x2_ReadAllVoltages(void)
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (int x = 0; x < 16; x++){//Reads all cell voltages
    hbms.CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  hbms.Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  hbms.Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  hbms.LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}

uint16_t BQ769x2_ReadCurrent(void)
// Reads PACK current
{
  uint16_t current = 0;
  DirectCommands(CC2Current, 0x00, R);
  //current = ((hbms.RX_data[1]<<8) + hbms.RX_data[0]);   //current is reported in mA
  for(uint8_t i = 0; i < 2; i++)
    current = current + (hbms.RX_data[i] << (8 * i));
  return current;
}

float BQ769x2_ReadTemperature(uint8_t command)
{
  DirectCommands(command, 0x00, R);
  //RX_data is a global var
  return (0.1 * (float)(hbms.RX_data[1]*256 + hbms.RX_data[0])) - 273.15;  // converts from 0.1K to Celcius
}

void BQ769x2_ReadPassQ(void)
{ // Read Accumulated Charge and Time from DASTATUS6
  Subcommands(DASTATUS6, 0x00, R);
  hbms.AccumulatedCharge_Int = ((hbms.RX_32Byte[3]<<24) + (hbms.RX_32Byte[2]<<16) + (hbms.RX_32Byte[1]<<8) + hbms.RX_32Byte[0]); //Bytes 0-3
  hbms.AccumulatedCharge_Frac = ((hbms.RX_32Byte[7]<<24) + (hbms.RX_32Byte[6]<<16) + (hbms.RX_32Byte[5]<<8) + hbms.RX_32Byte[4]); //Bytes 4-7
  hbms.AccumulatedCharge_Time = ((hbms.RX_32Byte[11]<<24) + (hbms.RX_32Byte[10]<<16) + (hbms.RX_32Byte[9]<<8) + hbms.RX_32Byte[8]); //Bytes 8-11
}

// ********************************* End of BQ769x2 Measurement Commands   *****************************************
