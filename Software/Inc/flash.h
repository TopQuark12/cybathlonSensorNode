#ifndef FLASH_H
#define FLASH_H

#define FLASH_SECTOR            FLASH_SECTOR_11	    //Flash sector to use for parameter storage
#define FLASH_SECTOR_ADDR       (void *)0x080E0000	//Start address of the specified flash sector

extern uint8_t flashSaveFlag;
extern osThreadId flashSaveThreadHandle;
extern uint32_t flashSaveThreadBuffer[256];
extern osStaticThreadDef_t flashSaveThreadControlBlock;

typedef enum flashSaveStatus_e
{
    FLASH_SAVE_READY,                   //Parameters ready to be saved, last save was successful
    FLASH_SAVE,                         //Set by user to launch a parameter save operation
    FLASH_SAVING,                       //Parameter save in progress
    FLASH_SAVE_ERROR_UNLOCK,            //Flash error: Unable to unlock flash
    FLASH_SAVE_ERROR_ERASE,             //Flash error: Unable to erase specified flash sector
    FLASH_SAVE_ERROR_INVALID_DATA_TYPE, //flashParamEntry->dataType unrecognised
    FLASH_SAVE_ERROR_WRITE              //Flash error: Unable to write into specified flash sector
} flashSaveStatus_e;

typedef struct flashParamEntry_t
{
    void *data;         //pointer to variable to save from and load to
    uint32_t dataType;  //Assumes a value of FLASH_Type_Program
} flashParamEntry_t;

//void flashInit(void);
void flashSaveThreadFunction(const void *argument);

#endif //FLASH_H