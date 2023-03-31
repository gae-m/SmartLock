/*
 * keyboard.h
 *
 *  Created on: Jun 27, 2022
 *      Author:	Gaetano
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include "stm32f4xx_hal.h"

#define R0_PORT	GPIOA
#define R0_PIN GPIO_PIN_7
#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_6
#define R2_PORT GPIOC
#define R2_PIN GPIO_PIN_7
#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_4
#define C0_PORT	GPIOC
#define C0_PIN GPIO_PIN_9
#define C1_PORT GPIOC
#define C1_PIN GPIO_PIN_8
#define C2_PORT GPIOC
#define C2_PIN GPIO_PIN_6
#define C3_PORT GPIOC
#define C3_PIN GPIO_PIN_5


uint8_t findKey(GPIO_TypeDef* port_col, uint16_t GPIO_Pin_col);
uint8_t getKey(int row, int col);


#endif /* INC_KEYBOARD_H_ */
