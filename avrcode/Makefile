GCCFLAGS=-g -Os -Wall -mmcu=atmega328p
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt 
AVRDUDEFLAGS=-F -p atmega328p -c usbtiny
LINKOBJECTS=../libnerdkits/delay.o ../libnerdkits/lcd.o ../libnerdkits/uart.o

all:	i2c-upload

i2c.hex:	i2c.c
	make -C ../libnerdkits
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o i2c.o i2c.c -lm ${LINKOBJECTS}
	avr-objcopy -j .text -O ihex i2c.o i2c.hex
	
i2c.ass:	i2c.hex
	avr-objdump -S -d i2c.o > i2c.ass
	
i2c-upload:	i2c.hex
	avrdude ${AVRDUDEFLAGS} -U flash:w:i2c.hex:a
