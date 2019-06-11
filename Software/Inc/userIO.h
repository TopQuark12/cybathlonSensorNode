/*
 * userIO.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alex's Desktop
 */

#ifndef INC_USERIO_H_
#define INC_USERIO_H_

#define POLL_PERIOD_MS			2

#define BUTTON_MUTEX_TIMEOUT	osWaitForever

#define BUTTON_COMP_MASK	(uint32_t) 0xFF0000FF
#define BUTTON_COMP_PRES	(uint32_t) 0x000000FF
#define BUTTON_COMP_RELS	(uint32_t) 0xFF000000

typedef struct button_t
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint32_t history;
	osMutexId mutexID;
} button_t;

typedef struct buttonThreadParam_t
{
	uint8_t numButton;
	button_t *buttonArray;
} buttonThreadParam_t;

uint8_t isButtonPressed(button_t *button);
uint8_t isButtonReleased(button_t *button);
uint8_t isButtonDown(button_t *button);
uint8_t isButtonUp(button_t *button);

enum buttonAlias_e
{
	setCAN = 0,
	setSen
};

extern button_t buttons[2];
void buttonSetup(void);

#endif /* INC_USERIO_H_ */
