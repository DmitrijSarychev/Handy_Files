/*
 * MX25L4006EM_FLASH.c
 *
 *  Created on: Jul 16, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#include "MX25L4006EM_FLASH.h"
#include "spi.h"


#define MX25L4006EM_WRITE_IN_PROGRESS            0x01
#define MX25L4006EM_WRITE_ENABLE_LATCH           0x02

#define WRITE_ENABLE                        0x06
#define WRITE_DISABLE                       0x04
#define READ_STATUS_REG1                    0x05
#define READ_STATUS_REG2                    0x35
#define WRITE_STATUS_REG                    0x01
#define READ_DATA                           0x03
#define PAGE_PROGRAM                        0x02
#define SECTOR_ERASE                        0x20
#define CHIP_ERASE                          0xC7
#define ENABLE_RESET                        0x66
#define RESET                               0x99

#define MICROCHIP_ID                        0xC2
#define SPI_DEVICE_ID                       0x20
#define CAPACITY_64_MBIT                    0x13


void dummyData_Init(dummyData_t *dummy)
{
  dummy->data8 = 128;
  dummy->data16 = 32768;
  dummy->dataChar = 'a';
  dummy->dataFloat = 420.69;
  for(uint8_t i = 0; i < sizeof(dummy->dataArray); i++)
    dummy->dataArray[i] = i;
}

void dummyData_Process(dummyData_t *dummy)
{
  dummy->data8 = dummy->data8 + 1;
  dummy->data16 = dummy->data16 + 1;
  dummy->dataChar = dummy->dataChar + 1;
  dummy->dataFloat = dummy->dataFloat + 1;
  for(uint8_t i = 0; i < sizeof(dummy->dataArray); i++)
      dummy->dataArray[i]++;
}

HAL_StatusTypeDef MX25L4006EM_SPI_Transmit_Data(uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  status = HAL_SPI_Transmit(&hspi2, data, size, 1000);
  return status;
}

HAL_StatusTypeDef MX25L4006EM_SPI_Receive_Data(uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  status = HAL_SPI_Receive(&hspi2, data, size, 1000);
  return status;
}

uint8_t MX25L4006EM_Init(void)
{
  memory_info_t MX25L4006EM_memory;

  MX25L4006EM_ResetFlash();
  MX25L4006EM_get_JEDEC_ID(&MX25L4006EM_memory);

  if(MICROCHIP_ID == MX25L4006EM_memory.manufacturer_id
      && SPI_DEVICE_ID == MX25L4006EM_memory.memory_type)
    return HAL_OK;
  else
    return HAL_ERROR;

}

void MX25L4006EM_ResetFlash(void)
{
  uint8_t data_to_send[] = {ENABLE_RESET, RESET};

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(data_to_send, 1);
  MX25L4006EM_CS_HIGH;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(&data_to_send[1], 1);
  MX25L4006EM_CS_HIGH;
}

void MX25L4006EM_get_JEDEC_ID(memory_info_t *info)
{
  uint8_t data_to_send = 0x9F;
  uint8_t receive_data[] = {0,0,0};

  MX25L4006EM_CS_LOW;

  MX25L4006EM_SPI_Transmit_Data(&data_to_send, 1);
  MX25L4006EM_SPI_Receive_Data(receive_data, 3);

  MX25L4006EM_CS_HIGH;

  info->manufacturer_id = receive_data[0];
  info->memory_type = receive_data[1];
  info->capacity = receive_data[2];
}

void MX25L4006EM_WriteEnable(void)
{
  uint8_t data_to_send = WRITE_ENABLE;
  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(&data_to_send, 1);
  MX25L4006EM_CS_HIGH;
}

void MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch(void)
{
  while(!(MX25L4006EM_ReadStatusRegister1() & MX25L4006EM_WRITE_ENABLE_LATCH))
    {
      MX25L4006EM_WriteEnable();
    }
}

void MX25L4006EM_WaitForWriteEnableLatch(void)
{
  while(!(MX25L4006EM_ReadStatusRegister1() & MX25L4006EM_WRITE_ENABLE_LATCH))
  {
    ;
  }
}

void MX25L4006EM_WaitForWriteInProgressClear(void)
{
  while((MX25L4006EM_ReadStatusRegister1() & MX25L4006EM_WRITE_IN_PROGRESS))
  {
    ;
  }
}

HAL_StatusTypeDef MX25L4006EM_SectorErase(uint16_t sector_number)
{
  uint32_t adress;
  adress = sector_number * SECTOR_SIZE;
  uint8_t data_to_send[] = { 0, 0, 0, 0 };
  HAL_StatusTypeDef status;

  MX25L4006EM_WaitForWriteInProgressClear();
  MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch();

  data_to_send[0] = SECTOR_ERASE;
  data_to_send[1] = (adress >> 16) & 0xff;
  data_to_send[2] = (adress >> 8) & 0xff;
  data_to_send[3] = adress & 0xff;

  MX25L4006EM_CS_LOW;
  status = MX25L4006EM_SPI_Transmit_Data(data_to_send, 4);
  MX25L4006EM_CS_HIGH;

  MX25L4006EM_WaitForWriteInProgressClear();

  return status;
}

HAL_StatusTypeDef MX25L4006EM_ChipErase(void)
{
  uint8_t data_to_send =  CHIP_ERASE;
  HAL_StatusTypeDef status;

  MX25L4006EM_WaitForWriteInProgressClear();
  MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch();

  MX25L4006EM_CS_LOW;
  status = MX25L4006EM_SPI_Transmit_Data(&data_to_send, 1);
  MX25L4006EM_CS_HIGH;

  MX25L4006EM_WaitForWriteInProgressClear();

  return status;
}

HAL_StatusTypeDef MX25L4006EM_PageProgram(uint32_t page_adress, uint8_t *data, uint16_t size)
{
  uint8_t data_to_send[] = { 0, 0, 0, 0 };
  HAL_StatusTypeDef status;

  MX25L4006EM_WaitForWriteInProgressClear();
  MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch();

  data_to_send[0] = PAGE_PROGRAM;
  data_to_send[1] = (page_adress >> 16) & 0xff;
  data_to_send[2] = (page_adress >> 8) & 0xff;
  data_to_send[3] = page_adress & 0xff;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(data_to_send, 4);
  status = MX25L4006EM_SPI_Transmit_Data(data, size);
  MX25L4006EM_CS_HIGH;

  return status;
}

HAL_StatusTypeDef MX25L4006EM_ReadDataBytes(uint32_t adress, uint8_t *data, uint16_t size)
{
  uint8_t data_to_send[] = { 0, 0, 0, 0 };
  HAL_StatusTypeDef status;

  MX25L4006EM_WaitForWriteInProgressClear();

  data_to_send[0] = READ_DATA;
  data_to_send[1] = (adress >> 16) & 0xff;
  data_to_send[2] = (adress >> 8) & 0xff;
  data_to_send[3] = adress & 0xff;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(data_to_send, 4);
  status = MX25L4006EM_SPI_Receive_Data(data, size);
  MX25L4006EM_CS_HIGH;

  return status;
}

uint8_t MX25L4006EM_ReadStatusRegister1(void)
{
  uint8_t data_to_send = READ_STATUS_REG1;
  uint8_t receive_data = 0;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(&data_to_send, 1);
  MX25L4006EM_SPI_Receive_Data(&receive_data, 1);
  MX25L4006EM_CS_HIGH;

  return receive_data;
}

uint8_t MX25L4006EM_ReadStatusRegister2(void)
{
  uint8_t data_to_send = READ_STATUS_REG2;
  uint8_t receive_data = 0;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(&data_to_send, 1);
  MX25L4006EM_SPI_Receive_Data(&receive_data, 1);
  MX25L4006EM_CS_HIGH;

  return receive_data;
}

void MX25L4006EM_WriteStatusRegister(uint8_t reg1, uint8_t reg2)
{
  uint8_t data_to_send[] = { 0, 0, 0 };

  MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch();

  data_to_send[0] = WRITE_STATUS_REG;
  data_to_send[1] = reg1;
  data_to_send[2] = reg2;

  MX25L4006EM_CS_LOW;
  MX25L4006EM_SPI_Transmit_Data(data_to_send, 2);
  MX25L4006EM_CS_HIGH;
}














