/*
 * lcd_i2c.c
 *
 *  Created on: Jun 16, 2022
 */

#include "lcd_i2c.h"

void lcd_send(uint8_t d, uint8_t mask);
void lcd_send4(uint8_t d, uint8_t mask);
void lcd_write_char(uint8_t c);

I2C_HandleTypeDef* hi2c;

//extern UART_HandleTypeDef huart2;



void lcd_send_cmd(uint8_t cmd){
	uint8_t mask = LCD_BACKLIGHT;
	lcd_send(cmd,mask);

}

void lcd_write_char(uint8_t c){
	uint8_t mask = Rs | LCD_BACKLIGHT;
	lcd_send(c,mask);
}

void lcd_send(uint8_t d, uint8_t mask){
	uint8_t d_h = d & 0xf0;
	uint8_t d_l = (d << 4) & 0xf0;

	lcd_send4(d_h,mask);
	lcd_send4(d_l,mask);

}

void lcd_send4(uint8_t d, uint8_t mask){
	uint8_t data[2];
	uint8_t mask_en = mask | En;

	data[0] = d | mask;
	data[1] = d | mask_en;

	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, data, 2, HAL_MAX_DELAY);
//	HAL_Delay(1);
	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, data, 1, HAL_MAX_DELAY);
//	HAL_Delay(1);
}

//void print_hex(uint8_t x){
//	char c[6];
//	sprintf(c,"0x%.2x ",x);
//	HAL_UART_Transmit(&huart2, c, 6, HAL_MAX_DELAY);
//}
//
//int check_busy_flag(){
//	uint8_t data_send[2];
//	uint8_t data_rcv[2];
//
//	data_send[0] = Rw | LCD_BACKLIGHT;
//	data_send[1] = data_send[0] | En;
//
//	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, &data_send[0], 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(hi2c, LCD_I2C_SLAVE_ADDR, data_rcv, 1, HAL_MAX_DELAY);
//
//	print_hex(data_rcv[0]);
//
//	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, &data_send[1], 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(hi2c, LCD_I2C_SLAVE_ADDR, data_rcv, 1, HAL_MAX_DELAY);
//
//	print_hex(data_rcv[0]);
//
//	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, &data_send[0], 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(hi2c, LCD_I2C_SLAVE_ADDR, data_rcv, 1, HAL_MAX_DELAY);
//
//	print_hex(data_rcv[0]);
//
//	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, &data_send[1], 1, HAL_MAX_DELAY);
//	HAL_I2C_Master_Receive(hi2c, LCD_I2C_SLAVE_ADDR, data_rcv, 1, HAL_MAX_DELAY);
//
//	print_hex(data_rcv[0]);
//
//	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_SLAVE_ADDR, data_send, 1, HAL_MAX_DELAY);
//
//	print_hex(data_rcv[0]);
//
//	sprintf(c,"%#.2x %#.2x\n",data_rcv[0],data_rcv[1]);
//	HAL_UART_Transmit(&huart2, c, 10, HAL_MAX_DELAY);
//
//	HAL_UART_Transmit(&huart2, '\n', 1, HAL_MAX_DELAY);
//
//	return 0;
//
//}

void lcd_write(const char* s){
	while (*s) lcd_write_char(*s++);
}

void lcd_clear(){
	lcd_send_cmd(LCD_CLEARDISPLAY);

}

int lcd_set_cursor(uint8_t row, uint8_t col){
	if(row > 1 || col > 15) return -1;

	switch(row){
	case(0):
		lcd_send_cmd(LCD_SETDDRAMADDR | col);
		break;
	case(1):
		lcd_send_cmd(LCD_SETDDRAMADDR | 0x40 | col);
		break;
	default:
		return -1;
	}

	return 0;
}

void lcd_init(I2C_HandleTypeDef* handle){
	hi2c = handle;

	HAL_Delay(100);

	lcd_send4(0x30,0x08);
	HAL_Delay(5);
	lcd_send4(0x30,0x08);
	HAL_Delay(1);
	lcd_send4(0x30,0x08);
	HAL_Delay(10);
	lcd_send4(0x20,0x08);
	HAL_Delay(10);

	lcd_send_cmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
	HAL_Delay(2);
	lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF);
	HAL_Delay(2);
	lcd_send_cmd(LCD_CLEARDISPLAY);
	HAL_Delay(2);
	lcd_send_cmd(LCD_ENTRYMODESET | LCD_ENTRYRIGHT | LCD_ENTRYSHIFTDISABLE);
	HAL_Delay(2);
	lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	HAL_Delay(2);

}
