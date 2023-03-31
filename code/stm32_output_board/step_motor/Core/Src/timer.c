/*
 * timer.c
 *
 *  Created on: 25 giu 2022
 *      Author: biagi
 */
#include "timer.h"

void delay(double t) // 1 s --> t = 10000
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < t);
}

