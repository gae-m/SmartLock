/*
 * debug.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Donato
 */

#include "debug.h"

extern UART_HandleTypeDef huart2;

void print(char*str){
	for(int i = 0; i < strlen(str); i++) HAL_UART_Transmit(&huart2, (uint8_t*) &str[i], 1, HAL_MAX_DELAY);
}


void print_byte_to_hex(uint8_t byte){
	char c[6];
	sprintf(c,"0x%.2x ",byte);
	HAL_UART_Transmit(&huart2, (uint8_t*) c, 5, HAL_MAX_DELAY);
}

void print_lf(){
	HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
}

void print_error(){
	HAL_UART_Transmit(&huart2, (uint8_t*)"error\n", 6, HAL_MAX_DELAY);
}
