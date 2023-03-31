/*
 * stepper_motor.h
 *
 *  Created on: 25 giu 2022
 *      Author: Biagio
 */

#ifndef INC_STEPPER_MOTOR_H_
#define INC_STEPPER_MOTOR_H_

#include "stm32f3xx_hal.h"

#define stepsperrev 4096 //numero di steps per ogni giro --> steps_angle = 5.625° / 64 steps

void stepper_step_angle (float angle, int direction, int rpm);


void stepper_half_drive (int step);


void stepper_set_rpm (int rpm);


#endif /* INC_STEPPER_MOTOR_H_ */
