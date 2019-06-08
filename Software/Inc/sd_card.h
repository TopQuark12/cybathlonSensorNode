/**
 * @file    sd_card.h
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node SD card
 * @version 0.1
 * @date	2019-6-8
 *
 * @copyright Copyright (c) 2019
 *
 */
 
#ifndef __SD_CARD_H__
#define __SD_CARD_H__

#include "sdio.h"
#include "ICM20602.h"

#define ERR_MOUNT_MKFS (1)
#define ERR_OPEN       (2)

extern uint8_t err;

void sd_test(void);
void fatfsThreadFunc(void const * argument);
uint8_t fatfsWriteIMUFrame(imuDataFrame_t inFrame);

#endif

