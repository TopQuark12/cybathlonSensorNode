/**
 * @file    userIO.c
 * @author	Alex Wong Tat Hang (thwongaz@connect.ust.hk)
 * @brief   Driver for the cybathlon sensor node onboard buttons and LEDs
 * @version 0.1
 * @date	2019-6-10
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "userIO.h"

//Thread definitions
TaskHandle_t buttonUpdateThreadHandle;
uint32_t buttonUpdateThreadStack[256];
StaticTask_t buttonUpdateThreadTCB;

//Button definitions
button_t buttons[2];

/**
 * @brief	Shift current button status into history buffer
 * 			Make this function external and poll periodically if not using RTOS
 * @param	button		pointer to button_t struct
 */
void updateButton(button_t *button)
{
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	button->history = button->history << 1;
	button->history |= !HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
	osMutexRelease(button->mutexID);
}

uint8_t isButtonPressed(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
    uint8_t pressed = 0;
    if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
        pressed = 1;
        button->history = 0xFFFFFFFF;
    }
    osMutexRelease(button->mutexID);
    return pressed;
}

uint8_t isButtonReleased(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	uint8_t released = 0;
	if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
		released = 1;
		button->history = 0x00000000;
	}
	osMutexRelease(button->mutexID);
	return released;
}

uint8_t isButtonDown(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	uint8_t state = button->history == 0xFFFFFFFF;
	osMutexRelease(button->mutexID);
	return state;
}

uint8_t isButtonUp(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	uint8_t state = button->history == 0x00000000;
	osMutexRelease(button->mutexID);
	return state;
}

void buttonUpdateThreadFunc(void const *argument)
{
	for (;;)
	{
		for (uint8_t i = 0; i < ((buttonThreadParam_t *)argument)->numButton; i++)
		{
			updateButton(&((buttonThreadParam_t *)argument)->buttonArray[i]);
		}
		osDelay(POLL_PERIOD_MS);
	}
}

void startButtonUpdate(void *argument)
{
    osThreadStaticDef(buttonUpdateThread, buttonUpdateThreadFunc, osPriorityNormal, 0, 256, buttonUpdateThreadStack, &buttonUpdateThreadTCB);
    buttonUpdateThreadHandle = osThreadCreate(osThread(buttonUpdateThread), argument);
}

void buttonSetup(void)
{
	buttons[setCAN].GPIOx = BUTTON_CAN_GPIO_Port;
	buttons[setCAN].GPIO_Pin = BUTTON_CAN_Pin;
	buttons[setCAN].history = 0;
	static osStaticMutexDef_t buttonCANCB;
	osMutexStaticDef(buttonCAN, &buttonCANCB);
	buttons[setCAN].mutexID = osMutexCreate(osMutex(buttonCAN));

	buttons[setSen].GPIOx = BUTTON_SEN_GPIO_Port;
	buttons[setSen].GPIO_Pin = BUTTON_SEN_Pin;
	buttons[setSen].history = 0;
	static osStaticMutexDef_t buttonSenCB;
	osMutexStaticDef(buttonSen, &buttonSenCB);
	buttons[setSen].mutexID = osMutexCreate(osMutex(buttonSen));

	static buttonThreadParam_t buttonParam;
	buttonParam.numButton = 2;
	buttonParam.buttonArray = buttons;

	startButtonUpdate(&buttonParam);
}

