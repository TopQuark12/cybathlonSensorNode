#ifndef FLASH_H
#define FLASH_H

#define FLASH_SECTOR            FLASH_SECTOR_11	    //Flash sector to use for parameter storage
#define FLASH_SECTOR_ADDR       0x080E0000	        //Start address of the specified flash sector

extern osThreadId flashSaveThreadHandle;
extern uint32_t flashSaveThreadBuffer[256];
extern osStaticThreadDef_t flashSaveThreadControlBlock;

//Status enumeration for the flash module
typedef enum 
{
    FLASH_SAVE_READY = 0x00U,                   //Parameters ready to be saved, last save was successful
    FLASH_SAVE = 0x01U,                         //Set by user to launch a parameter save operation
    FLASH_SAVING = 0x02U,                       //Parameter save in progress
    FLASH_SAVE_ERROR_UNLOCK = 0x03U,            //Flash error: Unable to unlock flash
    FLASH_SAVE_ERROR_ERASE = 0x04U,             //Flash error: Unable to erase specified flash sector
    FLASH_SAVE_ERROR_INVALID_DATA_TYPE = 0x05U, //flashParamEntry->dataType unrecognised
    FLASH_SAVE_ERROR_WRITE = 0x06U              //Flash error: Unable to write into specified flash sector
} flashSaveStatus_e;

//Structure for parameters needed to be stored in flash
typedef struct flashParamEntry_t
{
    void *data;         //pointer to variable to save from and load to
    uint32_t dataType;  //Assumes a value of FLASH_Type_Program
} flashParamEntry_t;

extern flashSaveStatus_e gFlashSaveFlag;
void flashLoad(const flashParamEntry_t *paramList);
void flashSaveThreadFunction(const void *argument);
void startFlashSave(void *argument);

#endif //FLASH_H