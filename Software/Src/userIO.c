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
osStaticMutexDef_t buttonCANCB;
osStaticMutexDef_t buttonSenCB;
buttonThreadParam_t buttonParam;

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
	button->history |= HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
	osMutexRelease(button->mutexID);
}

uint8_t is_button_pressed(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
    uint8_t pressed = 0;
    if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
        pressed = 1;
        button->history = 0xFFFFFFFF;
    }
    return pressed;
    osMutexRelease(button->mutexID);
}

uint8_t is_released(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	uint8_t released = 0;
	if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
		released = 1;
		button->history = 0x00000000;
	}
	return released;
	osMutexRelease(button->mutexID);
}

uint8_t is_button_down(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	return (button->history == 0xFFFFFFFF);
	osMutexRelease(button->mutexID);
}

uint8_t is_button_up(button_t *button){
	osMutexWait(button->mutexID, BUTTON_MUTEX_TIMEOUT);
	return (button->history == 0x00000000);
	osMutexRelease(button->mutexID);
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
	osMutexStaticDef(buttonCAN, &buttonCANCB);
	buttons[setCAN].mutexID = osMutexCreate(osMutex(buttonCAN));

	buttons[setSen].GPIOx = BUTTON_SEN_GPIO_Port;
	buttons[setSen].GPIO_Pin = BUTTON_SEN_Pin;
	buttons[setSen].history = 0;
	osMutexStaticDef(buttonSen, &buttonSenCB);
	buttons[setSen].mutexID = osMutexCreate(osMutex(buttonSen));

	buttonParam.numButton = 2;
	buttonParam.buttonArray = buttons;

	startButtonUpdate(&buttonParam);
}

