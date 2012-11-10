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
void init_LCDports(void);
void LCD_print (char *string);
void LCD_position (unsigned char row, unsigned char col);
void dis_cmd(unsigned char cmd);
void dis_data(unsigned char data);
