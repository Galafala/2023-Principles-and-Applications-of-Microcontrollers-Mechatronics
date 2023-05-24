#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

uint16_t ADC0Read(const int channel);

uint16_t ADC1Read(const int channel);

void serialOutput();

void USART_putstring(char* StringPtr);

int main(void) {
	CLKPR=(1<<CLKPCE);
	CLKPR=0b00000011; // set clk to 1Mhz
	DDRD=0xFF; // PORTD as output
	DDRB=0xFF; // PORTB as output
	
    DDRC = 0;
	TCCR0A=0b10100001; // phase correct PWM
	TCCR0B=0b00000010; // timer prescaler
	TCCR2A=0b10100001; // phase correct PWM
	TCCR2B=0b00000010; // timer prescaler
	
	serialOutput();


	while(1) {				
		ADCSRA |= (1<<ADEN);

		float sv0 = 0;
		float sv1 = 0;

		sv0 = (float)ADC0Read(3);
		sv1 = (float)ADC1Read(4);

		CLKPR=0b10000000; // set timer for serial output
		CLKPR=0b00000000; // set timer for serial output

		char Buffer[8];
		USART_putstring("DMS: ");
		
		char *intStr = itoa((float) sv0, Buffer, 10);
		strcat(intStr, ", IR: ");
		USART_putstring(intStr);

		intStr = itoa((float) sv1, Buffer, 10);
		strcat(intStr, "\n");
		USART_putstring(intStr);

		_delay_ms(500);
	}

	return 0;
}

uint16_t ADC0Read(const int channel) {
	ADMUX = 0b01000011;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

uint16_t ADC1Read(const int channel) {
	ADMUX = 0b0100100;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

void serialOutput() {
	unsigned int BaudR = 9600;
	unsigned int ubrr = (F_CPU / (BaudR*16UL))-1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<TXEN0);
}

void USART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00){
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *StringPtr;
		StringPtr++;
	}
}