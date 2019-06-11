/**
 * @file    sd_card.c
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node SD card
 * @version 0.1
 * @date	2019-6-8
 *
 * @copyright Copyright (c) 2019
 *
 */
 
#include "bsp_driver_sd.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "sdio.h"
#include "sd_card.h"
#include "main.h"
#include "ICM20602.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "userIO.h"
#include "FreeRTOS.h"

FATFS          SDFatFs;                               /* file system object for SD card logical drive */
uint32_t       byteswritten, bytesread;               /* file write/read counts */
uint8_t        rtext[100];                            /* file read buffer */
uint8_t        err;
static char    *f_name = "data.csv";           /* file name */
uint8_t        wtext[] = " Welcome to HKUST! "; 	/* file write buffer */

extern QueueHandle_t imuDataQueueHandle;

/**
  * @brief  doing some tests to SD card, like mount, create file, open a text etc. 
  * @param  
  * @retval 
  */
void sd_test(void)
{   
	/* Register the file system object to the FatFs module */
	if ((f_mount(&SDFatFs, (TCHAR const *)SDPath, 0) == FR_OK))
	{

			/* Create and Open a new text file object with write access */
			if (f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)  
			{
				/* Write data to the text file */
				f_write(&SDFile, wtext, sizeof(wtext), (void *)&byteswritten);  
				/* Close the open text file */
				f_close(&SDFile); 
				
				/* Open the text file object with read access */
				if (f_open(&SDFile, f_name, FA_READ) == FR_OK)  
				{
					/* Read data from the text file */
					f_read(&SDFile, rtext, sizeof(rtext), (UINT*)&bytesread);  
					
					/* Close the open text file */
					f_close(&SDFile);  
				}
				else
				{
					err=ERR_OPEN;
				}
			}
			else
			{
				err=ERR_OPEN;
			}
	}
	else
	{
		err=ERR_MOUNT_MKFS;
	}
	/* Unlink the SD disk I/O driver */
	FATFS_UnLinkDriver(SDPath);  
}

void fatfsStartLogging(void)
{
	/* Register the file system object to the FatFs module */
	if ((f_mount(&SDFatFs, (TCHAR const *)SDPath, 0) == FR_OK))
	{
		/* Create and Open a new text file object with write access */
		if (f_open(&SDFile, f_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			err = ERR_OPEN;
		}
	}
	else
	{
		err = ERR_MOUNT_MKFS;
	}
}

void fatfsEndLogging(void)
{
	/* Close the open text file */
	f_close(&SDFile);

	/* Unlink the SD disk I/O driver */
	FATFS_UnLinkDriver(SDPath);
}

void fatfsWriteln(char data[], uint8_t len)
{
	f_write(&SDFile, data, len, (void *)&byteswritten);
}

uint8_t fatfsWriteIMUFrame(imuDataFrame_t const *inFrame)
{
	char line[100];
	float data[2];
	char dataStr[2][30];
	char axis;
	const char axisName[] = {'x', 'y', 'z'};

	data[ACCEL] = rawConvertionAccel((int16_t *)&(inFrame->dataRaw[ACCEL]));
	char *tmpSign = (data[ACCEL] < 0) ? "-" : "";
	float tmpVal = (data[ACCEL] < 0) ? -data[ACCEL] : data[ACCEL];
	int tmpInt1 = tmpVal;
	float tmpFrac = tmpVal - tmpInt1;
	int tmpInt2 = trunc(tmpFrac * 10000);
	sprintf(dataStr[ACCEL], "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);

	data[GYRO] = rawConvertionGyro((int16_t *)&(inFrame->dataRaw[GYRO]));
	tmpSign = (data[GYRO] < 0) ? "-" : "";
	tmpVal = (data[GYRO] < 0) ? -data[GYRO] : data[GYRO];
	tmpInt1 = tmpVal;
	tmpFrac = tmpVal - tmpInt1;
	tmpInt2 = trunc(tmpFrac * 10000);
	sprintf(dataStr[GYRO], "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);

	axis = axisName[inFrame->dataType];
	uint8_t len = snprintf(line, 100, "%lu,%c,%u,%sg,%sdps\n", inFrame->timeStamp,  inFrame->node, axis, dataStr[ACCEL], dataStr[GYRO]);
	if (len >= 100) {
		return 1;		//write fail, over-sized line
	}

	fatfsWriteln(line, len);

	return 0;
}

void fatfsThreadFunc(void const * argument)
{
	MX_FATFS_Init();
//	fatfsStartLogging();
//	fatfsEndLogging();
	uint8_t shouldLog = 0;
	uint8_t shouldLogPrev = 0;
	char header[] = "Time stamp (ms), node, axis, acceleration (g), rotational velocity (degree per second)\n";

	for(;;)
	{
		if (isButtonPressed(&buttons[setSen])) {
			shouldLog = 1 - shouldLog;
			HAL_GPIO_WritePin(LED_G_SEN_GPIO_Port, LED_G_SEN_Pin, shouldLog);
		}

		if (shouldLog)							//Is logging turned on?
		{
			if (!shouldLogPrev)					//Has logging just been turned on?
			{
				fatfsStartLogging();
				fatfsWriteln(header, sizeof(header));
			}
			imuDataFrame_t IMUFrame;
			while (uxQueueMessagesWaiting(imuDataQueueHandle))
			{
				xQueueReceive(imuDataQueueHandle, &IMUFrame, 0);
				fatfsWriteIMUFrame(&IMUFrame);
			}
		}
		else									//logging is turned off
		{
			if (shouldLogPrev)					//Has logging just been turned off?
			{
				fatfsEndLogging();
			}
		}

		shouldLogPrev = shouldLog;
		osDelay(1);
	}
}





