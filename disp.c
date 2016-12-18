// LCD display routines
// by USHIRODA, Atsushi

#include <avr/io.h>
#include "disp.h"

#define USART_BAUD 9600

// output a character on LCD through USART
void outCharUsart(unsigned char c)
{	
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = c;
}

// display a string to LCD
void dispStr(char cmd, char *str)
{
	int i;
	
	outCharUsart(cmd);
	for (i=0; str[i] != 0; i++)
		outCharUsart(str[i]);
}

char selectButton(void)
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

void dispInit(void) {

  	const unsigned short baud = (F_CPU / (16UL * USART_BAUD)) - 1;

	// initialize USART
	UBRR0H = (unsigned char)(baud >> 8);
	UBRR0L = (unsigned char)baud;   
	UCSR0B = _BV(TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	dispStr(LCD_CLEAR, "");
}