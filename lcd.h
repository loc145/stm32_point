/*
 * lcd.h
 *
 *  Created on: Oct 11, 2020
 *      Author: LaTienLoc
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_
#include "stm32f0xx_hal.h"

// lcd_RS 	GPIOA	GPIO_PIN_9		//PA10
// lcd_E 	GPIOA	GPIO_PIN_10		//PA9
// lcd_D4 	GPIOA	GPIO_PIN_5		//PA5
// lcd_D5 	GPIOA	GPIO_PIN_6		//PA6
// lcd_D6 	GPIOA	GPIO_PIN_7		//PA7
// lcd_D7 	GPIOB	GPIO_PIN_1		//PB1

#define port_lcd_RS 	GPIOA
#define port_lcd_E 		GPIOA
#define port_lcd_D4 	GPIOA
#define port_lcd_D5 	GPIOA
#define port_lcd_D6 	GPIOA
#define port_lcd_D7 	GPIOB

#define pin_lcd_RS		GPIO_PIN_10
#define pin_lcd_E		GPIO_PIN_9
#define pin_lcd_D4		GPIO_PIN_5
#define pin_lcd_D5		GPIO_PIN_6
#define pin_lcd_D6		GPIO_PIN_7
#define pin_lcd_D7		GPIO_PIN_1


void LCD_Enable(void);
void LCD_Tach4bit(unsigned char Data);
void LCD_SendCommand(unsigned char cmd);
void LCD_Init(void);
void LCD_PutChar(unsigned char Data);
void LCD_Puts(char *s);
void LCD_GotoXY(int row, int col);
void LCD_Clear(void);

#endif /* INC_LCD_H_ */
