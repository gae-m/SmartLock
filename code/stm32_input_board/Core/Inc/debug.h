/*
 * debug.h
 *
 *  Created on: Jun 27, 2022
 *      Author: Donato
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

void print(char* str);

void print_byte_to_hex(uint8_t byte);

void print_lf();

void print_error();


#endif /* INC_DEBUG_H_ */
