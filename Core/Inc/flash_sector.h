/*
 * flash_sector.h
 *
 *  Created on: Feb 7, 2024
 *      Author: lucas
 */

#ifndef SRC_FLASH_SECTOR_H_
#define SRC_FLASH_SECTOR_H_

#ifndef INC_FLASH_SECTOR_F4_H_
#define INC_FLASH_SECTOR_F4_H_

#include "stdint.h"

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Convert_To_Str (uint32_t *Data, char *Buf);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM (uint32_t StartSectorAddress);


#endif /* INC_FLASH_SECTOR_F4_H_ */


#endif /* SRC_FLASH_SECTOR_H_ */
