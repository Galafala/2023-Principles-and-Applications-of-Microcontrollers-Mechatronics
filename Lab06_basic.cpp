#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/io.h>

void frequency(float f) {
	OCR0A=2*1000000/(f*64);		// n
	
	TCCR0B=0b00000011; 				// p=64, start Timer
	while ((TIFR0&(1<<OCF0A))==0);  // wait for flag TOV0=1
	TCCR0B=0; 					// stop Timer
	TIFR0= TIFR0|(1<<OCF0A); 		// clear TOV0
	PORTC=PORTC^0b0100000;
}

int main(void)
{
	CLKPR=(1<<CLKPCE);
	CLKPR=0b00000000; // set clk to 1Mhz

	TCCR0A=0x02; // CTC mode, int clk

	DDRB=0xFF; //7 segment
	PORTB=0xFF;

	DDRC=0b0100000; //Buzzer

	DDRD=0; // 1~8 button
	PORTD=0xFF;

	float g4 = 392.00;
	float a4 = 440.00;
	float b4 = 493.88;
	float c5 = 523.25;
	float d5 = 587.33;
	float e5 = 659.25;
	float f5 = 698.46;
	float g5 = 783.99;

	while(1) {
		if(~PIND&(1<<0)){
			frequency(g4);
			PORTB=0b10000010;//G4
		}
		if(~PIND&(1<<1)){
			frequency(a4);
			PORTB=0b10001000;//A4
		}
		if(~PIND&(1<<2)){
			frequency(b4);
			PORTB=0b10000000;//B4
		}
		if(~PIND&(1<<3)){
			frequency(c5);
			PORTB=0b11000110;//C5
		}
		if(~PIND&(1<<4)){
			frequency(d5);
			PORTB=0b11000000;//D4
		}
		if(~PIND&(1<<5)){
			frequency(e5);
			PORTB=0b10000110;//E5
		}
		if(~PIND&(1<<6)){
			frequency(f5);
			PORTB=0b10001110;//F5
		}
		if(~PIND&(1<<7)){
			frequency(g5);
			PORTB=0b10000010;//G5
		}
	}
}
