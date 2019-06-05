/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "ICM20602.h"
#include "string.h"

TaskHandle_t canTxThreadHandle;
uint32_t canTxThreadStack[256];
StaticTask_t canTxThreadTCB;

CAN_HandleTypeDef hcan1;
CAN_FilterTypeDef canAllPassFilter =
{
  0,
  0,
  0,
  0,
  CAN_FILTER_FIFO0,
  0,
  CAN_FILTERMODE_IDMASK,
  CAN_FILTERSCALE_32BIT,
  ENABLE,
  14
};

CAN_RxHeaderTypeDef canRxFrame;
uint8_t canRxBuffer[8];

CAN_TxHeaderTypeDef canTxFrame;
uint8_t canTxBuffer[8];
uint32_t *canTxMailboxUsed;
uint32_t canDefaultID = 0x200;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void canTxThreadFunc(void const *argument) 
{
  HAL_CAN_ConfigFilter(&hcan1, &canAllPassFilter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
}

void startCanTx(void *argument)
{
  //canTxThreadHandle = xTaskCreateStatic((TaskFunction_t)canTxThreadFunc, "canTxThread", 256, NULL, 3, canTxThreadStack, &canTxThreadTCB);
  osThreadStaticDef(canTxThread, canTxThreadFunc, osPriorityNormal, 0, 256, canTxThreadStack, &canTxThreadTCB);
  canTxThreadHandle = osThreadCreate(osThread(canTxThread), argument);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canRxFrame, canRxBuffer);
    canRxHandler();
  }
}

void canIDInit(void)
{
  canTxFrame.StdId = canDefaultID;
  canTxFrame.IDE = CAN_ID_STD;
  canTxFrame.RTR = CAN_RTR_DATA;
  canTxFrame.DLC = 8;
  canTxFrame.TransmitGlobalTime = DISABLE;
}

void canRxHandler(void)
{
  return;
}

void canTxMessageWithID(uint32_t canID, uint8_t data[])
{
  canTxFrame.StdId = canID;
  memcpy(canTxBuffer, data, 8);
  if (HAL_CAN_AddTxMessage(&hcan1, &canTxFrame, canTxBuffer, canTxMailboxUsed) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
}

void canTxMessage(uint8_t data[])
{
  canTxMessageWithID(canDefaultID, data);
}

void canTxFloatMessageWithID(uint32_t canID, float f1, float f2)
{
  memcpy(canTxBuffer, &f1, 4);
  memcpy(canTxBuffer + 4, &f2, 4);
  canTxMessageWithID(canID, canTxBuffer);
}

void canTxFloatMessage(float f1, float f2)
{
  canTxFloatMessageWithID(canDefaultID, f1, f2);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
