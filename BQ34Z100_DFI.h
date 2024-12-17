/*
 * BQ34Z100_DFI.h
 *
 *  Created on: Sep 19, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_BQ34Z100_DFI_H_
#define INC_BQ34Z100_DFI_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define BQ34Z100_DFI 0x16 //BQ34Z100 ROM adress according to "Going to Production with the bq43z100.pdf"

typedef struct
{
  uint8_t DataFlashImmage[32][36];
  uint8_t yIFRowData[512][100];
  uint8_t iRow;
  uint8_t Register66;
  uint32_t Checksum;

}BQ34DFI_t;

extern BQ34DFI_t hbq34z100_DFI;

HAL_StatusTypeDef bq34z100_dfi_fullAccess(void);
HAL_StatusTypeDef bq34z100_dfi_romMode(void);
HAL_StatusTypeDef bq34z100_dfi_exit_romMode(void);
HAL_StatusTypeDef bq34z100_dfi_writeCommand(uint8_t command, uint8_t * pData, uint8_t len);
HAL_StatusTypeDef bq34z100_dfi_readCommand(uint8_t reg_address, uint8_t * data, uint8_t len);
HAL_StatusTypeDef bq34z100_dfi_writeIF(void);
uint16_t bq34z100_dfi_calculate_checksum(uint8_t * pData, uint16_t len);

HAL_StatusTypeDef bq34z100_dfi_writeFlashImmage(void);

/*
 * This function is used for erasing the 2 rows of 96-byte Instruction Flash.
 * Fuction needs to be executed in UNEALED, FULL ACCESS, ROM MODE. For it to properly work, these
 * modes need to me accessed sequentially.
 * Function returns a HAL_StatusTypedef value indicating of the success/failure of the function.
 */
HAL_StatusTypeDef bq34z100_dfi_IF_erase(void);

/*
 * This function is used for mass erasing the internal FLASH memory of the BQ34Z100-G1, making it ready to receive
 * a new immage file.
 * Fuction needs to be executed in UNEALED, FULL ACCESS, ROM MODE. For it to properly work, these
 * modes need to me accessed sequentially.
 * Function returns a HAL_StatusTypedef value indicating of the success/failure of the function.
 */
HAL_StatusTypeDef bq34z100_dfi_flash_mass_erase(void);

/*
 * This function sends the 32byte rows of data to the chip.
 * Fuction needs to be executed in UNEALED, FULL ACCESS, ROM MODE. For it to properly work, these
 * modes need to me accessed sequentially.
 * Function returns a HAL_StatusTypedef value indicating of the success/failure of the function.
 */
HAL_StatusTypeDef bq34z100_dfi_write_flash(void);


/*
 * 00 05 - unknown command seen in the Golden Immage .bq and .df files
 * There is no documentation thet mentions it's function
 */
HAL_StatusTypeDef bq34z100_dfi_05_command(void);

/*
 * 00 08 - unknown command seen in the Golden Immage .bq and .df files
 * There is no documentation thet mentions it's function
 */
HAL_StatusTypeDef bq34z100_dfi_08_command(void);

/*
 * This command and data is written to the SOC at the end of the .bq file.
 * There is no documentation thet mentions it's function
 */
HAL_StatusTypeDef bq34z100_dfi_whatever_this_is(void);
#endif /* INC_BQ34Z100_DFI_H_ */
