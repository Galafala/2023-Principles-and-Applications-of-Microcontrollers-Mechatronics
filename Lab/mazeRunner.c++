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

void motorInit();

void serialOutput();

void USART_putstring(char* StringPtr);

void PIDcontrol(float* error, float* Kp, float* Ki, float* Kd, const float* Tp, float* integral, float* lastError, float* derivative);

int main(void) {
	motorInit();

	// float Kp = 220;
	// float Ki = 0.35;
	// float Kd = 10;

	float Kp = 225;
	float Ki = 0.15;
	float Kd = 0;

	const float Tp = 255;

	float integral = 0;
	float lastError = 0;
	float derivative = 0;

	while(1) {
       
		ADCSRA |= (1<<ADEN);

		float sv0 = 0;
		float sv1 = 0;

		sv0 = (float)ADC0Read(3); // DMS
		sv1 = (float)ADC1Read(4); // IR

		float error = 0;
		error += sv0;
		error += sv1;
		
        error = 1/error;
        
		if (sv0>500) {
			error *= -1;
		}
			
		PIDcontrol(&error, &Kp, &Ki, &Kd, &Tp, &integral, &lastError, &derivative);
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
	ADMUX = 0b01000100;
	ADMUX |= channel;
	ADCSRA |= (1<<ADSC) | (1<<ADIF);
	while ( (ADCSRA & (1<<ADIF)) == 0);
	ADCSRA &= ~(1<<ADSC);
	return ADC;
}

void motorInit() {
	CLKPR=(1<<CLKPCE);
	CLKPR=0b00000011; // set clk to 1Mhz
	DDRD=0xFF; // PORTD as output
	DDRB=0xFF; // PORTB as output
	
    DDRC = 0;
	TCCR0A=0b10100001; // phase correct PWM
	TCCR0B=0b00000010; // timer prescaler
	TCCR2A=0b10100001; // phase correct PWM
	TCCR2B=0b00000010; // timer prescaler
}

void PIDcontrol(float* error, float* Kp, float* Ki, float* Kd, const float* Tp, float* integral, float* lastError, float* derivative) {
	*integral += *error;
	*derivative = *error-*lastError;
	
	float turn = *Kp**error + *Ki**integral + *Kd**derivative; // revice turn

    float right = *Tp+turn;
    float left = *Tp-turn;

    if (right>255) right = 255;
	if (right<0) right = 0;

    if (left>255) left = 255;
    if (left<0) left = 0;

	OCR2B = right; // PD3 lN4 MOTOR2正轉 (right)
	OCR0B = left; // PD5 lN2 MOTOR1正轉 (left)
	
	*lastError = *error;
}