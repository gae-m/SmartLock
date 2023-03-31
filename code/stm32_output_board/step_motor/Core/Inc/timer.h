/*
 * timer.h
 *
 *  Created on: 25 giu 2022
 *      Author: biagi
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32f3xx_hal.h"

extern TIM_HandleTypeDef htim2;

void delay(double t); // 1 s --> t = 10000

#endif /* INC_TIMER_H_ */
