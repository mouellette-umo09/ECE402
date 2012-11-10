CC=avr-gcc 
CFLAGS=-g -Wl,-u,vfprintf -lprintf_flt -lm -Os -Wall -mcall-prologues -mmcu=atmega16
OBJ2HEX=avr-objcopy
TARGET=ps2
ADFLAGS=-p m16 -c avrispmkII -P usb

OBJS=ps2.o LCD.o

.PHONY: fuses prog erase


prog : $(TARGET).hex $(TARGET).eeprom
	avrdude $(ADFLAGS) -V -U flash:w:$(TARGET).hex:i
	avrdude $(ADFLAGS) -U eeprom:w:$(TARGET).eeprom:i

ps2.obj : ${OBJS}
	$(CC) $(CFLAGS) ${OBJS} -o ${TARGET}.obj

%.hex : %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.eeprom : %.obj
	$(OBJ2HEX) -j .eeprop -O ihex $< $@

LCD.o : LCD.h


erase :
	avrdude $(ADFLAGS) -E noreset -e
clean :
	rm -f *.hex *.obj *.o

fuses:
	avrdude $(ADFLAGS) -U lfuse:w:0xC4:m #http://www.engbedded.com/cgi-bin/fc.cgi
	avrdude $(ADFLAGS) -U hfuse:w:0xD9:m 
