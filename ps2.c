#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom16.h>
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
void init_PWM(void);
void moveMotor(void);
int readController(void);
int checkDpad(int button, int dpad);
void dPadTurn(int dpad);
void displayTurn(int dpad, int turn);
int checkTurn(int button, int turn);


int byte4;
int byte5;
int byte6;
int byte7;
int byte8;
int byte9;

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
	init_PWM();

	int dpad=6;
	int button;
	int turn=0;

	char buf2[70];

	//moveMotor();

	while (1)
	{
		button=readController();
		_delay_ms(1000);
		
		dpad=checkDpad(button,dpad);
		_delay_ms(500);
		
		turn=checkTurn(button,turn);
		_delay_ms(500);

		dPadTurn(dpad);
		_delay_ms(500);

		displayTurn(dpad,turn); 
		_delay_ms(500);
	}
	
	
	return 0;
}

int checkDpad(int button, int dpad)
{
	char buf[70];

	if (button==126)
	{
		dpad--;
	}

	if (button==158)
	{
		dpad++;
	}

	if (dpad>12)
		dpad=12;

	if (dpad<0)
		dpad=0;

	//sprintf(buf,"Dpad: %d , button: %d, \n\r",dpad,button);
    //my_send_string(buf);

	return dpad;
}

int checkTurn(int button, int turn)
{
	if (button==126)
        turn=1;
	else if (button==158)
        turn=2;
	else
		turn=0;

	return turn;
}

int readController(void)
{
    char buf[70];
    int temp;

    PORTC &= ~(1<<PSatt); //lowers ATT line which starts command sending 

    ps2_Communicate(0x01); //next 3 lines send Header bytes to controller   
    temp=ps2_Communicate(0x42); //will return controller type 
    ps2_Communicate(0x00); //lets AVR know controller is ready for data

    byte4=ps2_Communicate(0x00);
    byte5=ps2_Communicate(0x00); //reads first byte of data from controller
    byte6=ps2_Communicate(0x00); //reads second byte of data from controller
    byte7=ps2_Communicate(0x00);
    byte8=ps2_Communicate(0x00);
    byte9=ps2_Communicate(0x00); //reads first byte of data from controller
    
    sprintf(buf,"Data in byte4: %d , byte5: %d, byte6: %d, byte7: %d , byte8: %d, byte9: %d\n\r",byte4,byte5,byte6,byte7,byte8,byte9);
    my_send_string(buf);


    _delay_ms(50);
    PORTC |= (1<<PSatt); //raises ATT line after command is done sending
	
    return byte4;
}

void moveMotor()
{
	//OCR1A=100;

	//_delay_ms(4000);

	OCR1A=400;

	_delay_ms(2000);

	OCR1A=150;

	_delay_ms(2000);
	
	OCR1A=350;

	_delay_ms(2000);

	OCR1A=200;

	_delay_ms(2000);

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
	
	DDRD=0b00100000; //OCR1A as output for motor control	
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
	if (data==252||data==251)
		PORTD &= ~(1<<3);
	else
		PORTD |= (1<<3);


	//PB6 corresponds to 2nd bit in data
	if (data==248)
		PORTB &= ~(1<<6);
	else
		PORTB |= (1<<6);


	//PB7 corresponds to 3rd bit in data
	if (data==242)
		PORTB &= ~(1<<7);
	else
		PORTB |= (1<<7);


	//PD5 corresponds to 4th bit in data
	if (data==230)
		PORTD &= ~(1<<5);
	else
		PORTD |= (1<<5);


	//PD6 corresponds to 5th bit in data
	if (data==206)
		PORTD &= ~(1<<6);
	else
		PORTD |= (1<<6);


	//PD7 corresponds to 6th bit in data
	if (data==62)
		PORTD &= ~(1<<7);
	else
		PORTD |= (1<<7);


	//PB0 corresponds to 7th bit in data
	if (data==158)
		PORTB &= ~(1<<0);
	else
		PORTB |= (1<<0);


	//PD4 corresponds to 8th bit in data
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
			OCR1A=38;
			break;
		case 1:
            OCR1A=47;
            break;
		case 2:
            OCR1A=58;
            break;
		case 3:
            OCR1A=67;
            break;
		case 4:
            OCR1A=77;
            break;
		case 5:
            OCR1A=89;
            break;
		case 6:
            OCR1A=99;
            break;
		case 7:
            OCR1A=107;
            break;
		case 8:
            OCR1A=118;
            break;
		case 9:
            OCR1A=128;
            break;
		case 10:
            OCR1A=138;
            break;
		case 11:
            OCR1A=149;
            break;
		case 12:
            OCR1A=160;
            break;
	}
}

void displayTurn (int dpad, int turn)
{
	int i=0;

	if (turn==1)
	{
		for (i=0;i<3;i++)
		{
			PORTD=0b10111111;
			_delay_ms(250);
			PORTD=0b11111111;
			_delay_ms(250);
		}
	}

	if (turn==2)
    {
        for (i=0;i<3;i++)
        {
            PORTD=0b11011111;
            _delay_ms(250);
            PORTD=0b11111111;
            _delay_ms(250);
        }
    }	


	if (dpad<6)
		PORTD=0b01111111;
	if (dpad>6)
		PORTD=0b11101111;
	if (dpad==6)
		PORTD=0b11111111;

}

