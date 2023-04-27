#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRB=0xFF;
	DDRC=0;
	DDRD=0;

	PORTC=0xFF;
	PORTD=0xFF;
	
	PORTB=0b11111001;//1
	_delay_ms(500);
	PORTB=0b10100100;//2
	_delay_ms(500);
	PORTB=0b10110000;//3
	_delay_ms(500);
	PORTB=0b10011001;//4
	_delay_ms(500);
	PORTB=0b10010010;//5
	_delay_ms(500);
	PORTB=0b10000010;//6
	_delay_ms(500);
	PORTB=0b11011000;//7
	_delay_ms(500);
	PORTB=0b00000000;//8
	_delay_ms(500);
	PORTB=0b10010000;//9
	_delay_ms(500);
	PORTB=0b11000000;//0
	_delay_ms(500);
	
	while (1)
	{
		PIND=0xFF;
		PINC=0x7F;
		
		if(~PIND&0b00000001)
		{
			PORTB=0b11111001;//1
		}
		if(~PIND&0b00000010)
		{
			PORTB=0b10100100;//2
		}
		if(~PIND&0b00000100)
		{
			PORTB=0b10110000;//3
		}
		if(~PIND&0b0001000)
		{
			PORTB=0b10011001;//4
		}
		if(~PIND&0b00010000)
		{
			PORTB=0b10010010;//5
		}
		if(~PIND&0b00100000)
		{
			PORTB=0b10000010;//6
		}
		if(~PIND&0b01000000)
		{
			PORTB=0b11011000;//7
		}
		if(~PINC&0b0100000)
		{
			PORTB=0b00000000;//8
		}
		if(~PINC&0b0010000)
		{
			PORTB=0b10010000;//9
		}
		if(~PINC&0b0001000)
		{
			PORTB=0b11000000;//0
		}
	}
}

