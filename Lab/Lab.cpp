#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

uint16_t ADCRead(const int);
void USART_putstring(char* StringPtr);

const double k1 = 20.72084301;
const double k2 = -0.91152188;		

void Display1(int a);
void Display2(int b);

int main(void){
	CLKPR=0b10000000;
	CLKPR=0b00000000;
	DDRC = 0;
	ADCSRA |= (1<<ADEN);
	unsigned int BaudR = 19200;
	unsigned int ubrr = (F_CPU / (BaudR*16UL))-1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<TXEN0);
	
	DDRD=0xFF;
	DDRB=0xFF;
	PORTB = 0xFF;
	PORTD = 0xFF;
	
	
	while(1){
		double sumVal = 0;
		char Buffer[8];

		for(int i = 0; i < 100; i++) {
			sumVal += (double)ADCRead(0);
		}
			
		sumVal /= 100; //mean of 10 readings		
		
		int a,b;
		int t=0;
		double sensorVolt=sumVal*5/1024;

		t = pow(sensorVolt/k1, 1/k2);
		
		if(t<10)
		{a=0;b=t;}
		else {
			a=t/10;

			if((t*10)%10>4){
				b = t%10 +1;
			}
			else {
				b=t%10;
			}
		}
		Display1(a);
		Display2(b);
		
		char *intStr = itoa((int)sumVal, Buffer, 10);
		strcat(intStr, "\n");
		USART_putstring(intStr);
		_delay_ms(500);		
	}
}

uint16_t ADCRead(const int channel) {
	ADMUX = 0b01000000;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

void USART_putstring(char* StringPtr){
	while(*StringPtr != 0x00){
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *StringPtr;
		StringPtr++;
	}
}

void Display1(int a)
{
	if(a==0)
	{PORTD = 0b10000000;}
	if(a==1)
	{PORTD = 0b11110001;}
	if(a==2)
	{PORTD = 0b01001000;}
	if(a==3)
	{PORTD = 0b01100000;}
	if(a==4)
	{PORTD = 0b00110001;}
	if(a==5)
	{PORTD = 0b00100100;}
	if(a==6)
	{PORTD = 0b00000100;}
	if(a==7)
	{PORTD = 0b10110000;}
	if(a==8)
	{PORTD = 0b00000000;}
	if(a==9)
	{PORTD = 0b00100000;}
}

void Display2(int b)
{
	if(b==0)
	{PORTB = 0b01000000;}
	if(b==1)
	{PORTB = 0b01111001;}
	if(b==2)
	{PORTB = 0b00100100;}
	if(b==3)
	{PORTB = 0b00110000;}
	if(b==4)
	{PORTB = 0b00011001;}
	if(b==5)
	{PORTB = 0b00010010;}
	if(b==6)
	{PORTB = 0b00000010;}
	if(b==7)
	{PORTB = 0b01011000;}
	if(b==8)
	{PORTB = 0b00000000;}
	if(b==9)
	{PORTB = 0b00010000;}
}