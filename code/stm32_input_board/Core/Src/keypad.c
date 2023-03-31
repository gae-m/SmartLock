/*
 * keyboard.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Gaetano
 */
#include "keypad.h"


GPIO_TypeDef* portsR[4] = {R0_PORT,R1_PORT,R2_PORT,R3_PORT};
uint16_t pinsR[4] = {R0_PIN,R1_PIN,R2_PIN,R3_PIN};

GPIO_TypeDef* portsC[4] = {C0_PORT,C1_PORT,C2_PORT,C3_PORT};
uint16_t pinsC[4] = {C0_PIN,C1_PIN,C2_PIN,C3_PIN};

uint8_t keyMap[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


void resetRowPins(){
	for(int i = 0; i < 4; i++) HAL_GPIO_WritePin(portsR[i], pinsR[i], GPIO_PIN_RESET);
}

int scanRows(GPIO_TypeDef* port, uint16_t GPIO_Pin){
	int r = -1;

	while(r < 3 && HAL_GPIO_ReadPin(port, GPIO_Pin) == GPIO_PIN_RESET){
		r++;
		HAL_GPIO_WritePin(portsR[r], pinsR[r], GPIO_PIN_SET);
	}

	if(HAL_GPIO_ReadPin(port, GPIO_Pin) == GPIO_PIN_RESET) r = -1;

	resetRowPins();
	return r;
}

uint8_t getKey(int row, int col){
	return keyMap[row][col];
}

uint8_t findKey(GPIO_TypeDef* port_col, uint16_t GPIO_Pin_col){
	int row,col = 0;

	row = scanRows(port_col,GPIO_Pin_col);
	if(row == -1) return 0x00;

	while(GPIO_Pin_col != pinsC[col] && col < 4) col++;
	if(col >= 4) return 0x00;

	return getKey(row,col);
}





