/*
 * MX25L4006EM_FLASH.h
 *
 *  Created on: Jul 16, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_MX25L4006EM_FLASH_H_
#define INC_MX25L4006EM_FLASH_H_

#include "main.h"

#define PAGE_SIZE   256
#define SECTOR_SIZE 4096

#define MX25L4006EM_CS_LOW (HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, 0))
#define MX25L4006EM_CS_HIGH (HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, 1))

typedef struct
{
  uint8_t manufacturer_id;
  uint8_t memory_type;
  uint8_t capacity;
}memory_info_t;

typedef struct
{
  uint8_t   data8;
  uint16_t  data16;
  char      dataChar;
  float     dataFloat;
  uint8_t   dataArray[10];

}__attribute__((packed, aligned(1))) dummyData_t;

void dummyData_Init(dummyData_t *dummy);
void dummyData_Process(dummyData_t *dummy);

HAL_StatusTypeDef MX25L4006EM_SPI_Transmit_Data(uint8_t *data, uint16_t size);
HAL_StatusTypeDef MX25L4006EM_SPI_Receive_Data(uint8_t *data, uint16_t size);

uint8_t MX25L4006EM_Init(void);
void MX25L4006EM_ResetFlash(void);
void MX25L4006EM_get_JEDEC_ID(memory_info_t *info);
void MX25L4006EM_WriteEnable(void);
void MX25L4006EM_WriteEnable_and_WaitForWriteEnableLatch(void);
void MX25L4006EM_WaitForWriteEnableLatch(void);
void MX25L4006EM_WaitForWriteInProgressClear(void);
HAL_StatusTypeDef MX25L4006EM_SectorErase(uint16_t sector_number);
HAL_StatusTypeDef MX25L4006EM_ChipErase(void);
HAL_StatusTypeDef MX25L4006EM_PageProgram(uint32_t page_adress, uint8_t *data, uint16_t size);
HAL_StatusTypeDef MX25L4006EM_ReadDataBytes(uint32_t adress, uint8_t *data, uint16_t size);
uint8_t MX25L4006EM_ReadStatusRegister1(void);
uint8_t MX25L4006EM_ReadStatusRegister2(void);
void MX25L4006EM_WriteStatusRegister(uint8_t reg1, uint8_t reg2);


#endif /* INC_MX25L4006EM_FLASH_H_ */
