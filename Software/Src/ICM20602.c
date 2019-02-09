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
imuRaw_t gIMUOffset, imuRawData;

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
    //reset sensor
    icm20602WriteReg(PWR_MGMT_1, 0x80);
    osDelay(1000);
    //clear sleep bit to activate sensor
    icm20602WriteReg(PWR_MGMT_1, 0x01);
    //apply sensor filter setting
    icm20602WriteReg(CONFIG, GYRO_LPF_176HZ);
    icm20602WriteReg(ACCEL_CONFIG2, ACCEL_LPF_218_1);
    //apply sensor measurement range setting
    icm20602WriteReg(GYRO_CONFIG, ICM20602_GYRO_MEASUREMENT_RANGE << 3);    //shift by 3 bits, don't change
    icm20602WriteReg(ACCEL_CONFIG, ICM20602_ACCEL_MEASUREMENT_RANGE << 3);  //shift by 3 bits, don't change
    //apply sensor offset setting
    icm20602WriteReg(XG_OFFS_USRH, gIMUOffset.gyroData[xAxis] >> 8);
    icm20602WriteReg(XG_OFFS_USRL, gIMUOffset.gyroData[xAxis] & 0xFF);
    icm20602WriteReg(YG_OFFS_USRH, gIMUOffset.gyroData[yAxis] >> 8);
    icm20602WriteReg(YG_OFFS_USRL, gIMUOffset.gyroData[yAxis] & 0xFF);
    icm20602WriteReg(ZG_OFFS_USRH, gIMUOffset.gyroData[zAxis] >> 8);
    icm20602WriteReg(ZG_OFFS_USRL, gIMUOffset.gyroData[zAxis] & 0xFF);
    icm20602WriteReg(XA_OFFSET_H, gIMUOffset.accData[xAxis] >> 8);
    icm20602WriteReg(XA_OFFSET_L, gIMUOffset.accData[xAxis] & 0xFF);
    icm20602WriteReg(YA_OFFSET_H, gIMUOffset.accData[yAxis] >> 8);
    icm20602WriteReg(YA_OFFSET_L, gIMUOffset.accData[yAxis] & 0xFF);
    icm20602WriteReg(ZA_OFFSET_H, gIMUOffset.accData[zAxis] >> 8);
    icm20602WriteReg(ZA_OFFSET_L, gIMUOffset.accData[zAxis] & 0xFF);
    return 1;
}

/**
 * @brief	get gyro, accel and temp data from IMU
 */
void icm20602Update(void) 
{
    static uint8_t txBuffer[ICM20602_UPDATE_BUFFER_SIZE];
    static uint8_t rxBuffer[ICM20602_UPDATE_BUFFER_SIZE];
    for (uint8_t i = 0; i < ICM20602_UPDATE_BUFFER_SIZE - 1; i++)
    {
        txBuffer[i] = 0x80 | (i + ACCEL_XOUT_H);
    }
    icm20602TransmitReceive(txBuffer, rxBuffer, ICM20602_UPDATE_BUFFER_SIZE);
    imuRawData.accData[xAxis] = (rxBuffer[1]) << 8 | rxBuffer[2];
    gIMUdata.accData[xAxis] = imuRawData.accData[xAxis] / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    imuRawData.accData[yAxis] = (rxBuffer[3]) << 8 | rxBuffer[4];
    gIMUdata.accData[yAxis] = imuRawData.accData[yAxis] / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    imuRawData.accData[zAxis] = (rxBuffer[5]) << 8 | rxBuffer[6];
    gIMUdata.accData[zAxis] = imuRawData.accData[zAxis] / 32768.0 * (ICM20602_ACCEL_MEASUREMENT_RANGE + 1) * 2;
    imuRawData.tempData = (rxBuffer[7]) << 8 | rxBuffer[8];
    gIMUdata.tempData = imuRawData.tempData * ICM20602_TEMP_SENSITIVITY + ICM20602_TEMP_OFFSET;
    imuRawData.gyroData[xAxis] = (rxBuffer[9]) << 8 | rxBuffer[10];
    gIMUdata.gyroData[xAxis] = imuRawData.gyroData[xAxis] / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
    imuRawData.gyroData[yAxis] = (rxBuffer[11]) << 8 | rxBuffer[12];
    gIMUdata.gyroData[yAxis] = imuRawData.gyroData[yAxis] / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
    imuRawData.gyroData[zAxis] = (rxBuffer[13]) << 8 | rxBuffer[14];
    gIMUdata.gyroData[zAxis] = imuRawData.gyroData[zAxis] / 32768.0 * (ICM20602_GYRO_MEASUREMENT_RANGE + 1) * 250;
}
