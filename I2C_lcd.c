/*
lcdpcf8574 lib sample

copyright (c) Davide Gironi, 2013

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "lcdpcf8574.h"

#define UART_BAUD_RATE 9600
#include "uart.h"

int main(void)
{
	//init uart
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

	sei();

uint8_t charmap[8]={
	0b10000,
	0b00000,
	0b00100,
	0b00000,
	0b00000,
	0b00010,
	0b01000,
	0b00100,
};
int i = 0;
	//init lcd
	lcd_init(LCD_DISP_ON);
	
	lcd_command(0x40);
	for (i=0; i<8;i++){
		lcd_data(charmap[i]);
		}
	_delay_ms(10);
	charmap[0] =0b10001;
	

	lcd_command(0x48);
	for (i=0; i<8;i++){
		lcd_data(charmap[i]);
	}
	//lcd go home
	lcd_home();

	uint8_t led = 0;


	while(1) {
		lcd_led(led); //set led
		//led = !led; //invert led for next loop
	_delay_ms(1000);
	lcd_gotoxy(0, 0);
	lcd_data(0);
	lcd_data(1);
	_delay_ms(5000);
		//lcd_gotoxy(0, 0);
		//lcd_puts("Hello world!");
		lcd_gotoxy(0, 1);
		//test loop
		int i = 0;
		//int line = 0;
		lcd_puts("i= ");
		//lcd_gotoxy(3, 1);
		for(i=0; i<10; i++) {
			lcd_gotoxy(3, 1);
			char buf[10];
			itoa(i, buf, 10);
			//lcd_gotoxy(0, line);
			//lcd_puts("i= ");
			//lcd_gotoxy(3, line);
			lcd_puts(buf);
			//line++;
			//line %= 2;
			uart_puts(buf);
			uart_puts("\r\n");
			_delay_ms(1000);
		}
	}
}


