/*
 * userIO.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alex's Desktop
 */

#ifndef INC_USERIO_H_
#define INC_USERIO_H_

#define POLL_PERIOD_MS		2

#define BUTTON_COMP_MASK	(uint32_t) 0xFF0000FF
#define BUTTON_COMP_PRES	(uint32_t) 0x000000FF
#define BUTTON_COMP_RELS	(uint32_t) 0xFF000000

typedef struct button_t
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	uint32_t history;
} button_t;

typedef struct buttonThreadParam_t
{
	uint8_t numButton;
	button_t buttonArray[];
} buttonThreadParam_t;

uint8_t is_button_up(button_t *button);
uint8_t is_button_down(button_t *button);
uint8_t is_button_press(button_t *button);
uint8_t is_button_release(button_t *button);

#endif /* INC_USERIO_H_ */
