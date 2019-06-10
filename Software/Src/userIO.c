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

/**
 * @brief	Shift current button status into history buffer
 * 			Make this function external and poll periodically if not using RTOS
 * @param	button		pointer to button_t struct
 */
void updateButton(button_t *button)
{
	button->history = button->history << 1;
	button->history = HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
}

uint8_t is_button_pressed(button_t *button){
    uint8_t pressed = 0;
    if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
        pressed = 1;
        button->history = 0xFFFFFFFF;
    }
    return pressed;
}

uint8_t is_released(button_t *button){
	uint8_t released = 0;
	if ((button->history & BUTTON_COMP_MASK) == BUTTON_COMP_PRES){
		released = 1;
		button->history = 0x00000000;
	}
	return released;
}

uint8_t is_button_down(button_t *button){
	return (button->history == 0xFFFFFFFF);
}

uint8_t is_button_up(button_t *button){
	return (button->history == 0x00000000);
}


