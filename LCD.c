#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom16.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "LCD.h"


void init_LCD(void)
{
	dis_cmd(0x02); //sets in 4 bit mode
	_delay_ms(1);

	dis_cmd(0x28); //sets in 2 line mode
	_delay_ms(1);
	
	dis_cmd(0x80);  //turns on LCD
	_delay_ms(1);
}

void LCD_cmd(unsigned char cmd)
{
	LCD_DATA=cmd;
	
	ctrl=(0<<RS)|(0<<RW)|(1<<enable);
	_delay_ms(1);
	
	ctrl=(0<<RS)|(0<<RW)|(0<<enable);
	_delay_ms(50);
}

void LCD_data (unsigned char data)
{
	LCD_DATA=data;
	
	ctrl=(1<<RS)|(0<<RW)|(1<<enable);
	_delay_ms(1);
	
	ctrl=(1<<RS)|(0<<RW)|(0<<enable);
	_delay_ms(50);

}

void LCD_print (char *input)
{
	unsigned char i;
	
	while (input[i]!=0)
	{
		dis_data(input[i]);
		i++;
	}
}

void LCD_position (unsigned char row, unsigned char col)
{
	if (row==1)
	{
		row=0x80;
		dis_cmd(row+(col-1));
	}	
	
	if (row==2)
	{
		row=0xC0;
		dis_cmd(row+(col-1));
	}	
	
	
}

void dis_cmd(unsigned char cmd)
{
	char cmd1;
	
	cmd1=cmd&0xF0; //takes higher nibble to send to LCD for 4 bit mode
	LCD_cmd(cmd1);
	
	cmd1=(cmd<<4)&0xF0; //takes lower nibble to send to LCd
	LCD_cmd(cmd1);

}

void dis_data(unsigned char data)
{
	char data1;
	
	data1=data&0xF0; //takes higher nibble to send to LCD for 4 bit mode
	LCD_data(data1);
	
	data1=(data<<4)&0xF0; //takes lower nibble to send to LCd
	LCD_data(data1);


}

//sets up ports for LCD
void init_LCDports(void)
{
	DDRA|= 0b11110000; //data lines are all outputs
	DDRC = 0b11101011; //LCD control lines are outputs
	//LCD_DATA=0x00; //data originally set to 0
	//ctrl=0x00;
}

