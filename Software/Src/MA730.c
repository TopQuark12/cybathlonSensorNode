/**
 * @file    MA730.c
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node onboard MA730
 * @version 0.1
 * @date	2019-2-22
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
#include "MA730.h"
#include <string.h>

HAL_StatusTypeDef ma730TransmitReceive(uint16_t *pTxData, uint16_t *pRxdata, uint16_t size)
{
    HAL_StatusTypeDef state;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
    state = HAL_SPI_TransmitReceive(MA730_SPI_DRIVER, (uint8_t *)pTxData, (uint8_t *)pRxdata, size, MA730_TIMEOUT_TICKS);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
    return state;
}

uint8_t ma730ReadReg(uint8_t addr) 
{
    static uint16_t txBuffer[2];
    static uint16_t rxBuffer[2];
    txBuffer[0] = 0b010 << 13 | (addr & 0b00011111) << 8;
    txBuffer[1] = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    ma730TransmitReceive(txBuffer, rxBuffer, 2);
    return (uint8_t) rxBuffer[1] >> 8;
}

uint8_t ma730WriteReg(uint8_t addr, uint8_t data)
{
    static uint16_t txBuffer[2];
    static uint16_t rxBuffer[2];
    txBuffer[0] = 1 << 15 | (addr & 0b00011111) << 8 | data;
    txBuffer[1] = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));
    ma730TransmitReceive(txBuffer, rxBuffer, 2);
    return (uint8_t)rxBuffer[1] >> 8;
}

uint8_t ma730Init(void) 
{
    static uint16_t txBuffer[2];
    static uint16_t rxBuffer[2];
    txBuffer[0] = 0b010 << 13;
    //HAL_StatusTypeDef state;
    if (ma730TransmitReceive(txBuffer, rxBuffer, 2) != HAL_OK)
        return 0;
    return 1;
}

uint16_t ma730ReadAngle(void)
{
    static int16_t rxBuffer;
    static uint16_t txBuffer;
    txBuffer = 0;
    ma730TransmitReceive(&txBuffer, (uint16_t *)&rxBuffer, 1);
    return (rxBuffer + 16384);
}