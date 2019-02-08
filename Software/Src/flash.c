/**
 * @file    flash.c
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node parameter storage in flash
 * @version 0.1
 * @date	2019-2-6
 * 
 * @copyright Copyright (c) 2018
 * 
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "flash.h"
#include "ICM20602.h"

flashSaveStatus_e flashSaveFlag;
osThreadId flashSaveThreadHandle;
uint32_t flashSaveThreadBuffer[256];
osStaticThreadDef_t flashSaveThreadControlBlock;

const flashParamEntry_t savedParameters[] =
{
    {&gyroOffsetX, FLASH_TYPEPROGRAM_HALFWORD},
    {&gyroOffsetY, FLASH_TYPEPROGRAM_HALFWORD},
    {&gyroOffsetZ, FLASH_TYPEPROGRAM_HALFWORD},
    {&accelOffsetX, FLASH_TYPEPROGRAM_HALFWORD},
    {&accelOffsetY, FLASH_TYPEPROGRAM_HALFWORD},
    {&accelOffsetZ, FLASH_TYPEPROGRAM_HALFWORD},
    {NULL, 0}
};

FLASH_EraseInitTypeDef userSectorErase =
{
    FLASH_TYPEERASE_SECTORS, //Erase sectors instead of mass erase
    FLASH_BANK_1,            //Erase flash bank 1
    FLASH_SECTOR,            //Erase the last sector, sector 11 in F405RG is 128kbytes long and starts at 0x080E0000
    1,                       //Erase 1 sector only
    FLASH_VOLTAGE_RANGE_3    //Range 3 requires 2.7-3.6v, erases 32 bits at a time, takes about 1 sec for sector 11
};

// void flashInit(void) {

// }

uint8_t flashSave(const flashParamEntry_t *paramList, uint8_t *flag)
{
    static uint32_t flashDataTypeToSize[] =
    {
        1, 2, 4, 8
    };
    HAL_SuspendTick();
    static flashParamEntry_t *paramPtr;
    paramPtr = (flashParamEntry_t *) paramList;
    *flag = FLASH_SAVING;
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        *flag = FLASH_SAVE_ERROR_UNLOCK;
        return 1;   //Flash unlock error
    }
    
    static uint32_t sectorError = 0;
    //FLASH_WaitForLastOperation(10000);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (HAL_FLASHEx_Erase(&userSectorErase, &sectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        *flag = FLASH_SAVE_ERROR_ERASE;
        return 1; //Flash erase error
    }

    HAL_StatusTypeDef status = HAL_OK;
    static uint32_t flashAddress;
    static uint64_t tempData;
    flashAddress = FLASH_SECTOR_ADDR;
    // while ((paramList->data != NULL) && (status == HAL_OK))
    while ((status == HAL_OK))
    {
        if (!IS_FLASH_TYPEPROGRAM(paramPtr->dataType))
        {
            HAL_FLASH_Lock();
            *flag = FLASH_SAVE_ERROR_INVALID_DATA_TYPE;
            return 1; //Invalid dataType
        }
        memcpy(&tempData, paramPtr->data, flashDataTypeToSize[paramPtr->dataType]);
        status = HAL_FLASH_Program(paramPtr->dataType, flashAddress, tempData);
        paramPtr++;
        flashAddress = flashAddress + flashDataTypeToSize[paramPtr->dataType];
        if (paramPtr->data == NULL)
            break;
    }

    HAL_FLASH_Lock();
    if (status != HAL_OK) {
        *flag = FLASH_SAVE_ERROR_WRITE;
        return 1; //Flash write error
    }
    *flag = FLASH_SAVE_READY;
    HAL_ResumeTick();
    return 0;
}

void flashSaveThreadFunction(const void *argument)
{
    flashSaveFlag = FLASH_SAVE_READY;
    while(1)
    {
        osDelay(1);
        if(flashSaveFlag == FLASH_SAVE)
        {
            HAL_GPIO_WritePin(LED_G_SEN_GPIO_Port, LED_G_SEN_Pin, GPIO_PIN_SET);
            flashSave(savedParameters, &flashSaveFlag);
            HAL_GPIO_WritePin(LED_G_SEN_GPIO_Port, LED_G_SEN_Pin, GPIO_PIN_RESET);
        }
    }
}