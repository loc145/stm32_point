/*
 * lcd.c
 *
 *  Created on: Oct 11, 2020
 *      Author: LaTienLoc
 */
#include "lcd.h"


void LCD_Enable(void){
	HAL_GPIO_WritePin(port_lcd_E, pin_lcd_E, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(port_lcd_E, pin_lcd_E, GPIO_PIN_RESET);
	HAL_Delay(1);
}
void LCD_Tach4bit(unsigned char Data){
	HAL_GPIO_WritePin(port_lcd_D4, pin_lcd_D4, (Data & 1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(port_lcd_D5, pin_lcd_D5, ((Data>>1) & 1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(port_lcd_D6, pin_lcd_D6, ((Data>>2) & 1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(port_lcd_D7, pin_lcd_D7, ((Data>>3) & 1)? GPIO_PIN_SET:GPIO_PIN_RESET);
}

void LCD_SendCommand(unsigned char cmd){
	LCD_Tach4bit(cmd>>4);
	LCD_Enable();
	LCD_Tach4bit(cmd);
	LCD_Enable();
}
void LCD_Init(void){
	//Bu?c 1
	HAL_Delay(20);
	HAL_GPIO_WritePin(port_lcd_RS, pin_lcd_RS, GPIO_PIN_RESET);
	LCD_Tach4bit(0x03);
	LCD_Enable();
	//Bu?c 2
	HAL_Delay(10);
	LCD_Tach4bit(0x03);
	LCD_Enable();
	//Bu?c 3
	HAL_Delay(1);
	LCD_Tach4bit(0x03);
	LCD_Enable();
	//Bu?c 4
	HAL_Delay(1);
	LCD_Tach4bit(0x02);
	LCD_Enable();

	HAL_Delay(1);
	LCD_SendCommand(0x28); //Bu?c 5 - 6
	LCD_SendCommand(0x0c); //Bu?c 7 - 8
	LCD_SendCommand(0x06); //Bu?c 9 - 10
	LCD_SendCommand(0x01); //Bu?c 11 - 12
}
void LCD_PutChar(unsigned char Data){
	HAL_GPIO_WritePin(port_lcd_RS, pin_lcd_RS, GPIO_PIN_SET);
	LCD_SendCommand(Data);
	HAL_GPIO_WritePin(port_lcd_RS, pin_lcd_RS, GPIO_PIN_RESET);
}
void LCD_Puts(char *s){
   while (*s)
   {
	  LCD_PutChar(*s);
	  s++;
   }
}
void LCD_GotoXY(int row, int col){
	  unsigned char address;
	  if(!row)
		  address = (0x80 + col);
	  else
		  address = (0xc0 + col);

	  HAL_Delay(1);
	  LCD_SendCommand(address);
	  HAL_Delay(1);
}
void LCD_Clear(void){
	LCD_SendCommand(0x01);
	HAL_Delay(10);
}

