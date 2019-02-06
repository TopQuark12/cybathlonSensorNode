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

void flashInit(void) 
{
    
}