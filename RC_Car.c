#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

void init_PWM (void);
void displayData(int data); 
int ps2_Communicate(int output);
void init_serial(void);
void init_ports(void);
void putChar(char data);
char getChar(void);
void my_send_string(char * buf);
int getdPadPress (int dpad);
void dPadTurn (int dpad);

int byte4;
int byte5;
int byte6;
int byte7;
int dpad;
int mode;

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
	mode=0; //initially sets mode to zero to signify no selection
	dpad=6; //sets dpad to 6 initially which is home position (90 degrees)

	while (1)
	{
		PORTC &= ~(1<<PSatt); //lowers ATT line which starts command sending 
	
		ps2_Communicate(0x01); //next 3 lines send Header bytes to controller	
		temp=ps2_Communicate(0x42); //will return controller type 
		ps2_Communicate(0x00); //lets AVR know controller is ready for data

		byte4=ps2_Communicate(0x00); 
		byte5=ps2_Communicate(0x00); //reads first byte of data from controller
		byte6=ps2_Communicate(0x00); //reads second byte of data from controller
		byte7=ps2_Communicate(0x00);

		sprintf(buf,"Data in byte4: %d , byte5: %d, byte6: %d, b7: %d, controller: %d\n\r",byte4,byte5,byte6,byte7,temp);
        my_send_string(buf);

		
		_delay_ms(100);

		PORTC |= (1<<PSatt); //raises ATT line after command is done sending
	
		displayData(byte5); //lights LED's based on data in Byte 5
				
		_delay_ms(100);

		displayData(byte6-1); //lights LED's based on data in Byte 6

		_delay_ms(100);

	}
	
	return 0;
}

//initializes Ports for non inverting fast PWM w/ prescaler of 64
void init_PWM(void)
{
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11); //mode 14 needs wgm13,12,11 set (mode 14 is fast PWM)
													 //setting COM1A1 and COM1B1 sets the PWM as non inverting
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10); //setting CS11 & CS10 gives a prescaler of 64

	ICR1=2499; //value controls frequency of the PWM, value of 2499 sets frequency to 50Hz(20ms period)

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
	
	DDRA=0b11111111; //PortA	
	DDRC=0b11101011; //Sets ACK and Data to inputs, CMD CLK and ATT are outputs
	DDRD=0b00100000; //sets PD5 as output (servo motor connection)
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
	//PA0 corresponds to 1st bit in data
	if (data==252||data==251)
		PORTA &= ~(1<<0);
	else
		PORTA |= (1<<0);


	//PA1 corresponds to 2nd bit in data
	if (data==248)
		PORTA &= ~(1<<1);
	else
		PORTA |= (1<<1);


	//PA2 corresponds to 3rd bit in data
	if (data==242)
		PORTA &= ~(1<<2);
	else
		PORTA |= (1<<2);


	//PA3 corresponds to 4th bit in data
	if (data==230)
		PORTA &= ~(1<<3);
	else
		PORTA |= (1<<3);


	//PA4 corresponds to 5th bit in data
	if (data==206)
		PORTA &= ~(1<<4);
	else
		PORTA |= (1<<4);


	//PA5 corresponds to 6th bit in data
	if (data==62)
		PORTA &= ~(1<<5);
	else
		PORTA |= (1<<5);


	//PA6 corresponds to 7th bit in data
	if (data==158)
		PORTA &= ~(1<<6);
	else
		PORTA |= (1<<6);


	//PA7 corresponds to 8th bit in data
	if (data==126)
		PORTA &= ~(1<<7);
	else
		PORTA |= (1<<7);


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

//turns the servo motoring by changing OCR1A value based on value of dpad integer
void dPadTurn(int dpad)
{
	if (dpad<0)	//dpad must be between 0 and 12
		dpad=0;
	if (dpad>12)
		dpad=12;

	switch (dpad)
	{
		case 0:
			OCR1A=150;
			break;
		case 1:
            OCR1A=156;
            break;
		case 2:
            OCR1A=163;
            break;
		case 3:
            OCR1A=169;
            break;
		case 4:
            OCR1A=175;
            break;
		case 5:
            OCR1A=182;
            break;
		case 6:
            OCR1A=188;
            break;
		case 7:
            OCR1A=195;
            break;
		case 8:
            OCR1A=201;
            break;
		case 9:
            OCR1A=207;
            break;
		case 10:
            OCR1A=213;
            break;
		case 11:
            OCR1A=220;
            break;
		case 12:
            OCR1A=226;
            break;
	}
}









