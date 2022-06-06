/**************************************************************************//**
 * @file     EEPROM_Emulate.h
 * @brief    This is the header file of EEPROM_Emulate.c
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define FLASH_PAGE_SIZE             (FMC_FLASH_PAGE_SIZE)
#define FLASH_PROGRAM_SIZE_4BYTE    (4)
#define FLASH_PROGRAM_SIZE          (FLASH_PROGRAM_SIZE_4BYTE)
#define BANK1_IMAGE_READY           (0x5A5A3036)


#define EEP_ADDR_STARAT_IDX         (0x00)

/*
    JUMP FROM AP TO LDROM (0xA5)
    update F/W finish (0xBB)
*/
#define EEP_ADDR_MAGIC_TAG          (EEP_ADDR_STARAT_IDX)

/*
    BANK1 READY
    BANK1 CRC
    BANK1 LEN
    BANK1 START addr    
*/

#define EEP_ADDR_BANK1_READY        (EEP_ADDR_STARAT_IDX + sizeof(uint32_t)*1)
#define EEP_ADDR_BANK1_CRC          (EEP_ADDR_STARAT_IDX + sizeof(uint32_t)*2)
#define EEP_ADDR_BANK1_LEN          (EEP_ADDR_STARAT_IDX + sizeof(uint32_t)*3)
#define EEP_ADDR_BANK1_ADDR         (EEP_ADDR_STARAT_IDX + sizeof(uint32_t)*4)


#define DataFlash_BaseAddr			(FMC->DFBA)	//(FMC->DFBADR)
#define Max_Amount_of_Data			128UL
#define Status_Unwritten			0xFFFF

#define Even_Addr_Pos				16UL
#define Even_Addr_Mask				0xFF0000
#define Even_Data_Pos				24UL
#define Even_Data_Mask				0xFF000000
#define Odd_Addr_Pos				0UL
#define Odd_Addr_Mask				0xFF
#define Odd_Data_Pos				8UL
#define Odd_Data_Mask				0xFF00

#define Err_OverAmountData			0x01
#define Err_OverPageAmount			0x02
#define Err_ErrorIndex				0x03
#define Err_WriteBlockStatus		0x04

void FMC_Enable(void);
uint32_t Init_EEPROM(uint32_t data_size, uint32_t use_pages);
void Search_Valid_Page(void);
uint32_t Read_Data(uint8_t index, uint8_t *data);
uint32_t Write_Data(uint8_t index, uint8_t data);
void Manage_Next_Page(void);
uint16_t Get_Cycle_Counter(void);
