/**
 * @file    ICM20602.c
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node onboard ICM20602 
 * @version 0.1
 * @date	2019-2-2
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
#include "ICM20602.h"
#include <string.h>

imu_t gIMUdata;

/**
 * @brief	Basic bytewise SPI transmit and receive, with CS pin setting
 * @param	pTxData	    pointer to the transmit buffer
 * @param	pRxdata	    pointer to the receive buffer
 * @param	size	    number of bytes to transmit and receive
 * @return  HAL status, HAL_TIMEOUT	if no ICM_20602 is on board
 */
HAL_StatusTypeDef icm20602TransmitReceive(uint8_t *pTxData, uint8_t *pRxdata, uint16_t size)
{
    HAL_StatusTypeDef state;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, 0);
    state = HAL_SPI_TransmitReceive(ICM20602_SPI_DRIVER, pTxData, pRxdata, size, ICM20602_TIMEOUT_TICKS);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, 1);
    return state;
}

/**
 * @brief	read from the specified ICM20602 register
 * @param	address of the ICM20602 register to read from
 * @return	data byte read from the specified ICM20602 register
 */
uint8_t icm20602ReadReg(uint8_t addr)
{
    static uint8_t txBuffer[2];
    static uint8_t rxBuffer[2];
    txBuffer[0] = addr | 0b10000000; //set MSB indicating read operation
    icm20602TransmitReceive(txBuffer, rxBuffer, 2);
    return rxBuffer[1];
}

/**
 * @brief	write a byte into the specified ICM20602 register
 * @param	addr	address of the register to read from
 * @param	data	data to write into the specified register
 */
void icm20602WriteReg(uint8_t addr, uint8_t data)
{
    static uint8_t txBuffer[2];
    static uint8_t rxBuffer[2];
    txBuffer[0] = addr;
    txBuffer[1] = data;
    icm20602TransmitReceive(txBuffer, rxBuffer, 2);    
}

/**
 * @brief	initialize the ICM20602 sensor
 * @return	return 0 if ICM20602 not dected
 *          return 1 if ICM20602 detected and initialize
 */
uint8_t icm20602Init(void)
{
    if (icm20602ReadReg(WHO_AM_I) != 0x12) 
    {
        return 0;
    }
    icm20602WriteReg(PWR_MGMT_1, 1);       //clear sleep bit to activate sensor
    icm20602WriteReg(CONFIG, GYRO_LPF_176HZ);
    icm20602WriteReg(GYRO_CONFIG, ICM20602_GYRO_MEASUREMENT_RANGE << 3); //shift by 3 bits, don't change
    icm20602WriteReg(ACCEL_CONFIG, ICM20602_ACCEL_MEASUREMENT_RANGE << 3); //shift by 3 bits, don't change
    icm20602WriteReg(ACCEL_CONFIG2, ACCEL_LPF_218_1);
    return 1;
}

/**
 * @brief	get gyro, accel and temp data from IMU
 */
void icm20602Update(void) 
{
    static uint8_t txBuffer[ICM20602_UPDATE_BUFFER_SIZE];
    static uint8_t rxBuffer[ICM20602_UPDATE_BUFFER_SIZE];
    static int16_t temp;    //temporary storage
    for (uint8_t i = 0; i < ICM20602_UPDATE_BUFFER_SIZE - 1; i++)
    {
        txBuffer[i] = 0x80 | (i + ACCEL_XOUT_H);
    }
    icm20602TransmitReceive(txBuffer, rxBuffer, ICM20602_UPDATE_BUFFER_SIZE);
    temp = (rxBuffer[1]) << 8 | rxBuffer[2];
    gIMUdata.accData[xAxis] = temp / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    temp = (rxBuffer[3]) << 8 | rxBuffer[4];
    gIMUdata.accData[yAxis] = temp / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    temp = (rxBuffer[5]) << 8 | rxBuffer[6];
    gIMUdata.accData[zAxis] = temp / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    temp = (rxBuffer[7]) << 8 | rxBuffer[8];
    gIMUdata.tempData = temp * ICM20602_TEMP_SENSITIVITY + ICM20602_TEMP_OFFSET;
    temp = (rxBuffer[9]) << 8 | rxBuffer[10];
    gIMUdata.gyroData[xAxis] = temp / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
    temp = (rxBuffer[11]) << 8 | rxBuffer[12];
    gIMUdata.gyroData[yAxis] = temp / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
    temp = (rxBuffer[13]) << 8 | rxBuffer[14];
    gIMUdata.gyroData[zAxis] = temp / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
}
