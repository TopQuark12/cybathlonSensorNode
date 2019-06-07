/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

#define CAN_IMU_X_MASK 0x010
#define CAN_IMU_Y_MASK 0x020
#define CAN_IMU_Z_MASK 0x030

extern CAN_FilterTypeDef canAllPassFilter;

extern CAN_RxHeaderTypeDef canRxFrame;
extern uint8_t canRxBuffer[8];

extern CAN_TxHeaderTypeDef canTxFrame;
extern uint8_t canTxBuffer[8];
extern uint32_t *canTxMailboxUsed;
extern uint32_t canDefaultID;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void canIDInit(void);
void canRxHandler(void);
void canTxMessageWithID(uint32_t canID, uint8_t data[]);
void canTxMessage(uint8_t data[]);
void canTxFloatMessageWithID(uint32_t canID, float f1, float f2);
void canTxFloatMessage(float f1, float f2);
void startCanTx(void *argument);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
