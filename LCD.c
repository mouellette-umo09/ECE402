#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom16.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>


#define LCD_DATA PORTA
#define ctrl PORTC
#define enable PC7
#define RW PC6
#define RS PC5

#define LCDClr 0x01  //clears lcd
#define LCDEightbit 0x38; //sets up 8 bit data transfer for LCD
#define LCD_line1 0x80; //goes to first line of LCD
#define LCD_line2 0xC0; //goes to second line of LCD
#define cursorOn 0x0E; //turns cursor on

void LCD_cmd(unsigned char cmd);
void init_LCD(void);
void LCD_data(unsigned char data);
void init_ports(void);
void LCD_print (char *string);
void LCD_position (unsigned char row, unsigned char col);



int main(void)
{
	init_ports();
	
	_delay_ms(50);
	
	init_LCD();  //sets up LCD
	
	LCD_position(1,2);
	LCD_print("Desired");
	
	LCD_position(2,2);
	LCD_print("Actual");


	
	return 0;
}



void init_LCD(void)
{
	LCD_cmd(0x38);
	_delay_ms(1);
	
	
	LCD_cmd(0x01);
	_delay_ms(1);
	
	LCD_cmd(0x0E);
	_delay_ms(1);
	
	LCD_cmd(0x80);
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
		LCD_data(input[i]);
		i++;
	}
}

void LCD_position (unsigned char row, unsigned char col)
{
	if (row==1)
	{
		row=0x80;
		LCD_cmd(row+(col-1));
	}	
	
	if (row==2)
	{
		row=0xC0;
		LCD_cmd(row+(col-1));
	}	
	
	
}

//sets up ports for
void init_ports(void)
{
	ACSR |= (1<<ACD); //disables analog comparator (saves power)
	
	//DDRB=0b00000010; //PB7-PB6, PB0 are outputs for LED's
	//DDRD=0b11110000; //PD7-PD3 are outputs for LED's	
	//PORTD=0b11111111; //turns LEDS off initially
	DDRA=0b11111111; //data lines are all outputs
	DDRC = 0b11101011; //LCD control lines are outputs
	LCD_DATA=0x00; //data originally set to 0
	ctrl=0x00;
}

