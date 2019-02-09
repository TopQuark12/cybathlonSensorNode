/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       sd_card.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */
 
#include "bsp_driver_sd.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "sdio.h"
#include "sd_card.h"
#include "main.h"

FATFS          SDFatFs;                               /* file system object for SD card logical drive */
static uint8_t buffer[_MAX_SS];                       /* a work buffer for the f_mkfs() */
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
	/* Register the file system object to the FatFs module and create a FAT file system (format) on the logical drive */
	if ((f_mount(&SDFatFs, (TCHAR const *)SDPath, 0) == FR_OK) && (f_mkfs((TCHAR const *)SDPath, FM_ANY, 0, buffer, sizeof(buffer)) == FR_OK))
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



