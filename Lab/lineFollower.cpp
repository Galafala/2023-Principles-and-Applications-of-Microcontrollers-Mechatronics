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

uint16_t ADC2Read(const int channel);

void serialOutput();

void USART_putstring(char* StringPtr);

void PIDcontrol(float *error, float* Kp, float* Ki, float* Kd, const float* Tp, float* integral, float* lastError, float* derivative) {
	*integral += *error;
	*derivative = *error-*lastError;
	
	float turn = *Kp**error + *Ki**integral + *Kd**derivative;

    float right = *Tp+turn+50;
    float left = *Tp-turn;

    if (right>255) right = 255;
    if (left>255) left = 255;

	OCR2B = right; // PD3 lN4 MOTOR2正轉 (right)
	OCR0B = left; // PD5 lN2 MOTOR1正轉 (left)
	
	*lastError = *error;
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
	
	serialOutput();

	float Kp = 0.8;
	float Ki = 0;
	float Kd = 0;

	const float Tp = 255;
	const float offset = 700;

	float integral = 0;
	float lastError = 0;
	float derivative = 0;

	while(1) {
        CLKPR=0b10000000; // set timer for serial output
        CLKPR=0b00000000; // set timer for serial output
       
		ADCSRA |= (1<<ADEN);

		float sv0 = 0;
		float sv1 = 0;
		float sv2 = 0;

		sv0 = (float)ADC0Read(0);
		sv1 = (float)ADC1Read(1);
		sv2 = (float)ADC2Read(2);

		float error = 0;
		error += sv0;
		error += sv1;
		error += sv2;
		
		// error -= offset;
		if (sv2 > sv1) { // 偏左
			if (error > 0) error *= -1; 
		}
		else if (sv0 > sv1) { // 偏右
			if (error < 0) error *= -1; 
		}
		
		// PIDcontrol( &error, &Kp, &Ki, &Kd, &Tp, &integral, &lastError, &derivative);

		
        USART_putstring("sv0:");

        char Buffer[8];
        char *intStr = itoa((float) sv0, Buffer, 10);
        strcat(intStr, ", sv1:");
        USART_putstring(intStr);

        intStr = itoa((float) sv1, Buffer, 10);
        strcat(intStr, ", sv2:");
        USART_putstring(intStr);

        intStr = itoa((float) sv2, Buffer, 10);
        strcat(intStr, ", error:");
        USART_putstring(intStr);

        intStr = itoa((float) error, Buffer, 10);
        strcat(intStr, ", error-offset:");
        USART_putstring(intStr);

        intStr = itoa((float) error-offset, Buffer, 10);
        strcat(intStr, "\n");
        USART_putstring(intStr);

        _delay_ms(500);
	}

	return 0;
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