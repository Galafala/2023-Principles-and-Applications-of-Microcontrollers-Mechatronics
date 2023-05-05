#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>

void stop();

void turnleft();

void turnright();

void straight();

uint16_t ADC0Read(const int channel);

uint16_t ADC1Read(const int channel);

uint16_t ADC2Read(const int channel);

void serialOutput();

void USART_putstring(char* StringPtr);

inline void PIDcontrol(bool LFSensor[], float *Kp, float *Ki, float *Kd, float *offset, float *Tp, float *integral, float *lastError, float *derivative) {
	int error = 0;
	if     ((LFSensor[0]==false)&&(LFSensor[1]==false)&&(LFSensor[2]==true))   error = -2; // 偏左
	else if((LFSensor[0]==false)&&(LFSensor[1]==true) &&(LFSensor[2]==true))   error = -1; // 微偏左
	else if((LFSensor[0]==false)&&(LFSensor[1]==true) &&(LFSensor[2]==false))  error = 0;  // 居中
	else if((LFSensor[0]==true) &&(LFSensor[1]==true) &&(LFSensor[2]==false))  error = 1;  // 微偏右
	else if((LFSensor[0]==true) &&(LFSensor[1]==false)&&(LFSensor[2]==false))  error = 2;  // 偏右
	else if((LFSensor[0]==true) &&(LFSensor[1]==true) &&(LFSensor[2]==true))   error = 0;  // 全黑線

	*integral += error;
	*derivative = error-*lastError;
	
	float turn = *Kp*error + *Ki**integral + *Kd**derivative;

	OCR0B = *Tp+turn; // PD5 lN2 MOTOR1正轉 (right)
	OCR2B = *Tp-turn-3; // PD3 lN4 MOTOR2正轉 (left)

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
	
	serialOutput();

	float Kc = 4; // critical Kp
	float Pc = 3.53;

	float Kp = Kc*0.6;
	float Ki = 2*Kp/Pc;
	float Kd = Kp*Pc/8;

	Kp = 20;
	Ki = 0;
	Kd = 0;

	float offset = 450;
	float Tp = 50;

	float integral = 0;
	float lastError = 0;
	float derivative = 0;

	while(1) {			
		CLKPR=0b10000000; // set timer for serial output
		CLKPR=0b00000000; // set timer for serial output
				
		ADCSRA |= (1<<ADEN);

		bool LFSensor[3] = {0, 0, 0};

		float sv0 = 0;
		float sv1 = 0;
		float sv2 = 0;

		sv0 = (float)ADC0Read(0);
		sv1 = (float)ADC1Read(1);
		sv2 = (float)ADC2Read(2);

		int threshold = 400;
		if (sv0 > threshold) LFSensor[0] = true;
		else LFSensor[0] = false;
		if (sv1 > threshold) LFSensor[1] = true;
		else LFSensor[1] = false;
		if (sv2 > threshold) LFSensor[2] = true;
		else LFSensor[2] = false;
		
		PIDcontrol(LFSensor, &Kp, &Ki, &Kd, &offset, &Tp, &integral, &lastError, &derivative);

		char Buffer[8];
		char *intStr = itoa((int)sv1, Buffer, 10);
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