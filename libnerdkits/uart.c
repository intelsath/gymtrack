// uart.c
// for NerdKits with ATmega168, 14.7456 MHz clock
// mrobbins@mit.edu

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <inttypes.h>
#include "uart.h"

/* Define BAUD RATE (BAUD_PRESCALER) depending on the crystal used */
//#define BAUD_PRESCALER 7 // UBRRn value must be 7 for a baud rate of 115200 and a crystal of 14.7456MHZ according to datasheet (see table 19-9)
#define BAUD_PRESCALER 7 // UBRRn value must be 103 for a baud rate of 9600 and a crystal of 16MHZ according to datasheet (see table 19-9)

// Use a baud rate of 38400 if HC-05 bluetooth is used on AT mode

void uart_init() {
  // set baud rate
  /* The UBRRnH contains the four
    most significant bits, and the UBRRnL contains the eight least significant bits of the USART
    baud rate. From datasheet */
  UBRR0H = (uint16_t)(BAUD_PRESCALER>>8);
  UBRR0L = (uint16_t)(BAUD_PRESCALER);	// for 9600 with 16MHz clock
  // enable uart RX and TX
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  // set 8N1 frame format
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

  // set up STDIO handlers so you can use printf, etc
  fdevopen(&uart_putchar, &uart_getchar);
}


void uart_write(char x) {
  // wait for empty receive buffer
  while ((UCSR0A & (1<<UDRE0))==0);
  // send
  UDR0 = x;
}

uint8_t uart_char_is_waiting() {
  // returns 1 if a character is waiting
  // returns 0 if not
  return (UCSR0A & (1<<RXC0));
}

char uart_read() {
  // wait
  while(!uart_char_is_waiting());
  char x = UDR0;
  return x;
}

int uart_putchar(char c, FILE *stream) {
  uart_write(c);
  return 0;
}

int uart_getchar(FILE *stream) {
  int x = uart_read();
  return x;
}


