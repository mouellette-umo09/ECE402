#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/iom16.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include "LCD.h"


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
void init_A2D(void);
void init_timer2(void);
int getADC(void);
int checkSpeed(int button,int dirDpad);

volatile int overflows; //counts number of overflows
volatile int secondFlag;//flag that goes high every second
volatile int average; //past 16 averaged ADC values
volatile int finalAverage;//averaged ADC value to pass to PC
volatile int seconds;//time in seconds for timestamping data
double voltage;//voltage read to pass to PC
double temperature; //temp to pass to PC (from finalAverage)
volatile int samplecounter; //counts up to 16 for average of values
volatile int values[16];
double degree; //converts voltage ADC measurement to degree value
double desired; //holds desired degree

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
	//functions to initialize avr for all functions
	init_serial();
	init_ports();
	init_PWM();
	init_A2D();
	init_timer2();
	init_LCDports();
	
	_delay_ms(50);
	
	//initializes LCD for communication
	init_LCD();
	
	//sets up LCD initially with Desired on top line and Actual on second line
	LCD_position(1,2);
	LCD_print("d:");
	
	LCD_position(2,2);
	LCD_print("a:");
	
	_delay_ms(100);
	
	int dpad=7;			//determines the position of front motor
	int button;			//holds output from the controller
	int turn=0;
	int dirDpad=0;			//integer to hold current motor direction

	char buf2[70];
	char buf[100];
	
	samplecounter=0;
	voltage=0;
	temperature=0;
	finalAverage=0;
	seconds=0;//variable to hold the current second being passed
	secondFlag=0;
	degree=90;
	desired=90;
	
	
	int value;
	
	sei();

	moveMotor();

	while (1)
	{
		button=readController();
		_delay_ms(1000);
		
		if (button==126 || button==158)
		{
			dpad=checkDpad(button,dpad);
			_delay_ms(500);

			dPadTurn(dpad);
			_delay_ms(500);
		}
		
		if (button==206 || button==62)
		{
			dirDpad=checkSpeed(button,dirDpad);
		}

		//every second the LCD will update with the feedback ADC value
		if (secondFlag==1)
		{
			//converts the ADC value to a voltage, then a degree
			voltage=(((double)finalAverage)*1.94)/1024;
			degree=voltage*139.96-73.7;
			
			//send data serially
			//sprintf(buf,"%d,%d,%lf\n\r",seconds,finalAverage,voltage);
			//my_send_string(buf);
			
			//prints degree value to LCD
			sprintf(buf2,"a: %lf",degree);
			LCD_position(2,2);
			LCD_print(buf2);	

			secondFlag=0;	
		}
		
		//gets ADC value on each iteration
		value=getADC();			
			
	}
	
	
	return 0;
}

//reads data from controller, if up is pressed motor moves forward, if down is pressed motor moves backwards
//dirDpad=1 for forward, dirDpad=2 for reverse
int checkSpeed(int button, int dirDpad)
{
	char buf[70];
	
	//holds the desired motor direction based on controller input
	int nextDir=0;
	
	//if up is pressed, forward motor direction is wanted	
	if (button==206)
		nextDir=1;
		
	//if up is pressed, forward motor direction is wanted
	if (button==62)
		nextDir=2;
	
	//only changes motor direction and speed for proper controller inputs 
	if (nextDir==1 || nextDir==2)
	{	
		//indicates a desired change in direction
		if(nextDir!=dirDpad)
		{
			//both PWMs need to be reset to zero to avoid a short of the motor
			OCR1B=0;
			OCR0=0;
			
			_delay_ms(1000);
				
			//sets OCR1B to initial value to start the motor forwards
			if(nextDir==1)
			{
				OCR1B=1000;
				OCR0=0;
			}
			
			//sets OCR0 to initial value to start the motor in reverse	
			if(nextDir==2)
			{
				OCR0=180;
				OCR1B=0;
			}
			
		}
		//indicates a desired speed increase
		if(nextDir==dirDpad)
		{
			//increases speed forwards
			if (nextDir==1&&OCR1B<=200000)
				OCR1B+=250;
				
			//increases speed in reverse
			if (nextDir==2&&OCR0<=245)
				OCR0+=5;
		
		
		}
		
		//updates current direction in dirDpad
		dirDpad=nextDir;
	
	}


	//sprintf(buf,"dirDpad: %d , nextDir: %d, OCR1B: %d, OCR0: %d , button: %d\n\r",dirDpad,nextDir,OCR1B,OCR0,button);
    	//my_send_string(buf);
	
	return dirDpad;
}

//returns ADC value from pin A0 and averages the values to get more accurate results
int getADC()
{
	ADCSRA |= (1<<ADSC);
	
	while (!(ADCSRA & (1<<ADIF)));
	
	ADCSRA|=(1<<ADIF);
		
	int reading,i;
	reading=(ADCH<<8)+ADCL;//stores value from  A/D conversion

	samplecounter++;//increments through 16 values
	samplecounter&=0xF; //makes sure values go from 0-15

	values[samplecounter]=reading;//stores current ADC value
	average=0;//resets average to 0 for each interrupt

	for (i=0;i<16;i++)
	{
		average+=values[i];//adds up past 16 values
	}
	average>>=4; //divides by 16 to get average value
	
	finalAverage=average;  //stores the averaged value (it is volatile variable that can be accessed outside of the function)
	
	return ADC;
}

//checks returned value from controller to see if it is either a left or right dpad press for turning
int checkDpad(int button, int dpad)
{
	char buf[70];

	//decreases dpad variable if left dpad button is pressed
	if (button==126)
	{
		dpad--;
	}

	//increases dpad variable if right dpad button is pressed
	if (button==158)
	{
		dpad++;
	}

	//makes sure dpad variable is between 0 and 14
	if (dpad>14)
		dpad=14;

	if (dpad<0)
		dpad=0;

	//sprintf(buf,"Dpad: %d , button: %d, \n\r",dpad,button);
        //my_send_string(buf);

	return dpad;
}


//reads from the controller and returns the value
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
    
    //sprintf(buf,"Data in byte4: %d , byte5: %d, byte6: %d, byte7: %d , byte8: %d, byte9: %d\n\r",byte4,byte5,byte6,byte7,byte8,byte9);
    //my_send_string(buf);


    _delay_ms(50);
    PORTC |= (1<<PSatt); //raises ATT line after command is done sending
	
    return byte4;
}

//initializes the motor to be approx. 90 degrees
void moveMotor()
{
	//sets direction motor to approx 90 to start
	OCR1A=196;
	//turns back motor off initially
	OCR1B=0;
	OCR0=0;

}

//initializes Ports for non inverting fast PWM w/ prescaler of 64
void init_PWM(void)
{
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11); //mode 14 needs wgm13,12,11 set (mode 14 is fast PWM)
  								                      //setting COM1A1 and COM1B1 sets the PWM as non inverting
    TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10); //setting CS11 & CS10 gives a prescaler of 64

    ICR1=2499; //value controls frequency of the PWM, value of 2499 sets frequency to 50Hz(20ms period)
    
    //initializes Timer0 for PWM0
    TCCR0 |= (1<<CS01) | (1<<CS00) |(1<<WGM00) | (1<<WGM01) | (1<<COM01); //sets up fast non-inverting PWM with prescaler of 8
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


//sets up ports for use
void init_ports(void)
{	
	DDRD=0b00110000; //OCR1A and OCR1B as outputs for motor control		
	DDRC=0b11101011; //Sets ACK and Data to inputs, CMD CLK and ATT are outputs
	DDRB=0b00001000;  //sets OCR0 as output for h-bridge control
}

//sets up registers for adc conversions
void init_A2D(void)
{
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) ; //sets prescaler to 64 (125kHz for 8MHz clock)
	ADMUX = 0x00; //sets ADC reference voltage to AREF, and input to ADC0 (PA0)
	//ADCSRA |= (1<<ADIE); //enables interrupts for conversion
	ADCSRA |= (1<<ADEN); //enables conversions
	//ADCSRA |= (1<<ADATE); //enables free running mode to be used
	SFIOR = 0x00;        //sets ADC to free running mode
	ADCSRA |= (1<<ADSC); //starts conversions

}

//initializes flags for timer 2
void init_timer2(void)
{
	TCCR2 |= (1<<WGM21); //sets timer2 in CTC mode
	//TCCR2 |= (1<<CS22); //sets prescaler to 64(for 1MHz clock)
	TCCR2 |= (1<<CS22) | (1<<CS21); //sets prescaler to 256 (for 8MHz clock)
	TCNT2=0; //initializes counter
	TIMSK |= (1<<OCIE2); //enables compare match interrupt
	OCR2=125; //sets CTC compare value to 125
	overflows=0; //initializes overflow counter variable

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
	char buf2[70];
	
	if (dpad<0)	//dpad must be between 0 and 12
		dpad=0;
	if (dpad>14)
		dpad=14;

	switch (dpad)
	{
		//OCR1A value for 62 degrees
		case 0:
	    desired=62;
	    OCR1A=165;
	    LCD_position(1,2);
	    LCD_print("d: 62 ");
	    break;
	    	//OCR1A value for 66 degrees
		case 1:
	    desired=66;
	    LCD_position(1,2);
	    LCD_print("d: 66 ");
            OCR1A=170;
            break;
	    	//OCR1A value for 70 degrees
		case 2:
	    LCD_position(1,2);
	    desired=70;
	    LCD_print("d: 70 ");
            OCR1A=174;
            break;
	    	//OCR1A value for 74 degrees
		case 3:
	    LCD_position(1,2);
	    desired=74;
	    LCD_print("d: 74 ");
            OCR1A=178;
            break;
	    	//OCR1A value for 78 degrees
		case 4:
	    LCD_position(1,2);
	    desired=78;
	    LCD_print("d: 78 ");
            OCR1A=183;
            break;
	    	//OCR1A value for 82 degrees
		case 5:
	    LCD_position(1,2);
	    desired=82;
	    LCD_print("d: 82 ");
            OCR1A=187;
            break;
	    	//OCR1A value for 86 degrees
		case 6:
	    LCD_position(1,2);
	    desired=86;
	    LCD_print("d: 86 ");
            OCR1A=191;
            break;
	    	//OCR1A value for 90 degrees
		case 7:
	    LCD_position(1,2);
	    desired=90;
	    LCD_print("d: 90 ");
            OCR1A=195;
            break;
	    	//OCR1A value for 94 degrees
		case 8:
	    LCD_position(1,2);
	    desired=94;
	    LCD_print("d: 94 ");
            OCR1A=200;
            break;
	    	//OCR1A value for 98 degrees
		case 9:
            LCD_position(1,2);
	    desired=98;
	    LCD_print("d: 98 ");
            OCR1A=205;
            break;
	    	//OCR1A value for 102 degrees
		case 10:
	    LCD_position(1,2);
	    desired=102;
	    LCD_print("d: 102 ");
            OCR1A=210;
            break;
	    	//OCR1A value for 106 degrees
		case 11:
	    LCD_position(1,2);
	    desired=106;
	    LCD_print("d: 106 ");
            OCR1A=216;
            break;
	    	//OCR1A value for 110 degrees
		case 12:
	    LCD_position(1,2);
	    desired=110;
	    LCD_print("d: 110 ");
            OCR1A=222;
            break;
	    	//OCR1A value for 114 degrees
	    	case 13:
	    LCD_position(1,2);
	    desired=114;
	    LCD_print("d: 114 ");
            OCR1A=228;
            break;
	    	//OCR1A value for 118 degrees
	    	case 14:
	    LCD_position(1,2);
	    desired=118;
	    LCD_print("d: 118 ");
            OCR1A=233;
            break;
	}
}


//interrupt service routine to handle timing
ISR(TIMER2_COMP_vect)
{
	overflows++;
	//when 250 overflows are done then a second has passed
	//so data will be sent serially at this point
	if (overflows>=500)//should be 125(1MHz) 250 at 8MHz
	{
		finalAverage=average;
		secondFlag=1;
		overflows=0;
		seconds++;//counts up seconds for output to file
		
	}	

}



