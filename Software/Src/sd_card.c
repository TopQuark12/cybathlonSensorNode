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

FATFS          SDFatFs;                               /* file system object for SD card logical drive */
uint32_t       byteswritten, bytesread;               /* file write/read counts */
uint8_t        rtext[100];                            /* file read buffer */
uint8_t        err;
static char    *f_name = "cybathlon.txt";           /* file name */
uint8_t        wtext[] = " Welcome to HKUST! "; 	/* file write buffer */

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

void fatfsWriteIMUFrame(imuDataFrame_t inFrame)
{
	static uint8_t line[100];



}

void fatfsThreadFunc(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN fatfsThreadFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fatfsThreadFunc */
}





