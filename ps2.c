#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

void displayData(int data); 
int ps2_Communicate(int output);
void init_serial(void);
void init_ports(void);
void putChar(char data);
char getChar(void);
void my_send_string(char * buf);

int byte4;
int byte5;
int byte6;
int byte7;

/*Pins for Controller Wires
 *
 *Attention    PortC-Pin0
 *Clock        PortC-Pin1
 *Data         PortC-Pin2
 *Command      PortC-Pin3
 *Acknowledge  PortC-Pin4
 *
*/

//LEDS connected to PB0, PB6-PB7, PD3-PD7

#define PSatt 0   //PC0
#define PSclk 1   //PC1
#define PSdat 2   //PC2
#define PScmd 3   //PC3
#define PSack 4   //PC4
#define F_CPU 8000000


int main(void)
{
	init_serial();
	init_ports();

	char buf[70];
	int temp;
	while (1)
	{
		PORTC &= ~(1<<PSatt); //lowers ATT line which starts command sending 
	
		ps2_Communicate(0x01); //next 3 lines send Header bytes to controller	
		temp=ps2_Communicate(0x42); //will return controller type 
		ps2_Communicate(0x00); //lets AVR know controller is ready for data

		byte4=ps2_Communicate(0x00); //reads first byte of data from controller
		byte5=ps2_Communicate(0x00); //reads second byte of data from controller
		byte6=ps2_Communicate(0x00);
		byte7=ps2_Communicate(0x00);

		sprintf(buf,"Data in byte4: %d , byte5: %d, byte6: %d, b7: %d, controller: %d\n\r",byte4,byte5,byte6,byte7,temp);
        my_send_string(buf);

		
		_delay_ms(100);

		PORTC |= (1<<PSatt); //raises ATT line after command is done sending
	
		displayData(byte5); //lights LED's based on data in Byte 4
				
		_delay_ms(100);

		//PORTB=0b11000111;
		//PORTD=0b11100000;

		displayData(byte6-1); //lights LED's based on data in Byte 5

		_delay_ms(100);

		//PORTB=0b11000111;
        //PORTD=0b11100000;

	}
	
	return 0;
}


//initializes the serial communication for AVR
void init_serial(void)
{
	UBRRH=0;
    UBRRL=103;
    UCSRA=2; //sets baud to 9600 for 8MHz clock

    UCSRC= (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);// 8 BIT NO PARITY 2 STOP

    UCSRB=(1<<RXEN)|(1<<TXEN)  ; //ENABLE TX AND RX ALSO 8 BIT
}


//sets up ports for
void init_ports(void)
{
	ACSR |= (1<<ACD); //disables analog comparator (saves power)
	
	DDRB=0b11000001; //PB7-PB6, PB2-PB0 are outputs for LED's
	DDRD=0b11111000; //PD7-PD5 are outputs for LED's	
	
	DDRC=0b11101011; //Sets ACK and Data to inputs, CMD CLK and ATT are outputs

}


//sends data on command line and reads from data line
int ps2_Communicate(int output)
{
	int i;
	int data=0x00;

	for (i=0;i<8;i++)
	{
		//sets up command line for communication
		if(output & _BV(i))	
			PORTC |= (1<<PScmd);
		else
			PORTC &= ~(1<<PScmd);

		PORTC &= ~(1<<PSclk); //sets clock low, data read on falling edge 
		_delay_us(1);

		if (PINC & _BV(PSdat))
			data |= (1<<i); //stores data from PS2 DATA line

		PORTC |= (1<<PSclk); //sets clock back high
	}
	PORTC |= (1<<PScmd);//sets command line back high, end of communication

	return data;
}


//based on output from controller lights up LEDs
void displayData(int data)
{
	//turns on corresponding LED based on data from controller
	//PD3 corresponds to 1st bit in data
	//if(data & _BV(0))
	//	PORTD &= ~(1<<3);
	//else
	//	PORTD |= (1<<3);
	if (data==252||data==251)
		PORTD &= ~(1<<3);
	else
		PORTD |= (1<<3);


	//PB6 corresponds to 2nd bit in data
	//if(data & _BV(1))
	//	PORTB &= ~(1<<6);
	//else
	//	PORTB |= (1<<6);
	if (data==248)
		PORTB &= ~(1<<6);
	else
		PORTB |= (1<<6);


	//PB7 corresponds to 3rd bit in data
	//if(data & _BV(2))
	//	PORTB &= ~(1<<7);
	//else
	//	PORTB |= (1<<7);
	if (data==242)
		PORTB &= ~(1<<7);
	else
		PORTB |= (1<<7);


	//PD5 corresponds to 4th bit in data
	//if(data & _BV(3))
	//	PORTD &= ~(1<<5);
	//else
	//	PORTD |= (1<<5);
	if (data==230)
		PORTD &= ~(1<<5);
	else
		PORTD |= (1<<5);


	//PD6 corresponds to 5th bit in data
	//if(data & _BV(4))
	//	PORTD &= ~(1<<6);
	//else
	//	PORTD |= (1<<6);
	if (data==206)
		PORTD &= ~(1<<6);
	else
		PORTD |= (1<<6);


	//PD7 corresponds to 6th bit in data
	//if(data & _BV(5))
	//	PORTD &= ~(1<<7);
	//else
	//	PORTC |= (1<<7);
	if (data==62)
		PORTD &= ~(1<<7);
	else
		PORTD |= (1<<7);


	//PB0 corresponds to 7th bit in data
	//if(data & _BV(6))
	//	PORTB &= ~(1<<0);
	//else
	//	PORTB |= (1<<0);
	if (data==158)
		PORTB &= ~(1<<0);
	else
		PORTB |= (1<<0);


	//PD4 corresponds to 8th bit in data
	//if(data & _BV(7))
	//	PORTD &= ~(1<<4);
	//else 
	//	PORTD |= (1<<4);
	if (data==126)
		PORTD &= ~(1<<4);
	else
		PORTD |= (1<<4);


}


//puts a char to serial line
void putChar(char data)
{
    while (!(UCSRA&(1<<UDRE)));

    UDR=data;
}


//reads char from serial line
char getChar(void)
{
    while (!(UCSRA&(1<<RXC)));

    return UDR;
}


//sends an entire string serially
void my_send_string(char * buf)
{
    int x;  //uses software polling, assumes serial is set up  

    for(x=0;x<strlen(buf);x++)
    {
        putChar(buf[x]);
    }
}

