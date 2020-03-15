GCCFLAGS=-g -Os -Wall -mmcu=atmega328p 

all: uart.o I2C.o SSD1306.o Framebuffer.o

I2C.o: I2C.cpp
	avr-g++ ${GCCFLAGS} -o I2C.o -c I2C.cpp

SSD1306.o: SSD1306.cpp
	avr-g++ ${GCCFLAGS} -o SSD1306.o -c SSD1306.cpp

Framebuffer.o: Framebuffer.cpp
	avr-g++ ${GCCFLAGS} -o Framebuffer.o -c Framebuffer.cpp

uart.o: uart.c
	avr-gcc ${GCCFLAGS} -o uart.o -c uart.c