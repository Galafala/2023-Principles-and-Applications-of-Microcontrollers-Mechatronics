#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>

void stop() {
	OCR0A=0; // PD6 lN1 MOTOR1反轉right
	OCR0B=0; // PD5 lN2 MOTOR1正轉
	OCR2A=0;//PB3 lN3 MOTOR2反轉left
	OCR2B=0;//PD3 lN4 MOTOR2正轉
}

void turnleft() {	
	OCR0A=0; // PD6 lN1 MOTOR1反轉right
	OCR0B=80; // PD5 lN2 MOTOR1正轉
	OCR2A=0;//PB3 lN3 MOTOR2反轉left
	OCR2B=20;//PD3 lN4 MOTOR2正轉
}

void turnright() {
	OCR0A=0; // PD6 lN1 MOTOR1反轉right
	OCR0B=20; // PD5 lN2 MOTOR1正轉
	OCR2A=0;//PB3 lN3 MOTOR2反轉left
	OCR2B=80;//PD3 lN4 MOTOR2正轉
}

void straight() {
	OCR0A=0; // PD6 lN1 MOTOR1反轉right
	OCR0B=80; // PD5 lN2 MOTOR1正轉 (右輪)
	OCR2A=0;//PB3 lN3 MOTOR2反轉left
	OCR2B=85;//PD3 lN4 MOTOR2正轉 (左輪)
}

uint16_t ADC0Read(const int channel) {
	ADMUX = 0b01000000;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

uint16_t ADC1Read(const int channel) {
	ADMUX = 0b01000001;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

uint16_t ADC2Read(const int channel) {
	ADMUX = 0b010000010;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

void USART_putstring(char* StringPtr) {
	while(*StringPtr != 0x00){
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *StringPtr;
		StringPtr++;
	}
}

void PIDcontrol(float *sv1, float *sv2, float *sv3, float *Kp, float *Ki, float *Kd, float *offset, float *Tp, float *integral, float *lastError, float *derivative) {
	float error = *sv2-*offset;

	*integral += error;
	*derivative = error-*lastError;
	
	float turn = *Kp*error + *Ki**integral + *Kd**derivative;
	turn /= 100;

	OCR0B = *Tp+turn; // PD5 lN2 MOTOR1正轉
	OCR2B = *Tp-turn; // PD3 lN4 MOTOR2正轉

	*lastError = error;
}

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
	
	unsigned int BaudR = 9600;
	unsigned int ubrr = (F_CPU / (BaudR*16UL))-1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<TXEN0);
	
	float Kp = 1000;
	float Ki = 100;
	float Kd = 10000;
	float offset = 45;
	float Tp = 50;
	float integral = 0;
	float lastError = 0;
	float derivative = 0;

	while(1) {			
		// CLKPR=0b10000000;
		// CLKPR=0b00000000;
				
		ADCSRA |= (1<<ADEN);
		float sv1 = 0;
		float sv2 = 0;
		float sv3 = 0;
		for(int i = 0; i < 10; i++)	{
			sv1 += (float)ADC0Read(0);
			sv2 += (float)ADC1Read(1);
			sv3 += (float)ADC2Read(2);
		} // read data from sensors
		
		sv1 /= 10; // mean of 10 readings
		sv2 /= 10;
		sv3 /= 10;
		// PIDcontrol(&sv1, &sv2, &sv3, &Kp, &Ki, &Kd, &offset, &Tp, &integral, &lastError, &derivative);
		// char Buffer[40];
		
		// char *intStr = itoa((int)sv3, Buffer, 10);
		// strcat(intStr, "\n");
		// USART_putstring(intStr);
		// _delay_ms(500);

		// char *intStr = itoa((int)sv2, Buffer, 10);
		// strcat(intStr, "\n");
		// USART_putstring(intStr);
		// _delay_ms(500);

		// char *intStr = itoa((int)sv3, Buffer, 10);
		// strcat(intStr, "\n");
		// USART_putstring(intStr);
		// _delay_ms(500);		

		if(sv3>sv1) {
			turnleft();
			_delay_ms(20);
			stop();
			// _delay_ms(5);
		}
		else if (sv3<sv1) {
			turnright();
			_delay_ms(30);
			stop();
			// _delay_ms(5);
		}
		// e 
	}

	return 0;
}