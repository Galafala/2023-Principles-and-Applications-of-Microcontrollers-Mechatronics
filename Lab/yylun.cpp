// Define the microcontroller we are using
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>


#define SENSOR_THRES 500
#define CONTROL_THRES 255

//Map Sensor Number to ADC Channel
#define SENSOR1 0
#define SENSOR3 2
#define SENSOR5 1 // right

//Gloabal varriables
float pGain = 300;   //Proportional Gain
float iGain =  0;  //Integral Gain
float dGain =  100;  //Differential Gain
int rSpeed, lSpeed;

int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;      //Previous Error
float irSpeed=255, ilSpeed=255;
uint16_t L, R, M;

uint16_t ADCRead(const int);
float PID(float cur_value, int r_s, int l_s);
void MotorInit();

float control;
float s;

int main(void)
{
   //Initialize Motors subsystem.
   MotorInit();

    while(1)
    {        
      M =(float)ADCRead(SENSOR3);
      R =(float)ADCRead(SENSOR5);
      L =(float)ADCRead(SENSOR1);
      
      s = M + R + L;
      
      control = PID(s, R, L);

      rSpeed = irSpeed + control;
      lSpeed = ilSpeed - control;
      if (rSpeed > 255) {rSpeed = 255;}
      if (lSpeed > 255) {lSpeed = 255;}
      if (rSpeed < 100) {rSpeed = 0;}
      if (lSpeed < 100) {lSpeed = 0;}

      OCR2B=rSpeed;
      OCR0B=lSpeed;
    }
}

//Implements PID control
float PID(float cur_value, int r_s, int l_s)
{
  float pid;
  float error;
  int offset = 1160;
  error = offset / cur_value;
  if (r_s > 200)
  {
    error *= -1.4;
  }
  if (l_s > 200)
  {
    error *= 1.0;
  }
  eInteg += error;                  // integral is simply a summation over time
  ePrev = error; 
  pid = (pGain * error)  + (iGain * eInteg) + (dGain * (error - ePrev));

                     // save previous for derivative

  return pid;
}

void MotorInit(){
  CLKPR=(1<<CLKPCE);
 CLKPR=0b00000000;       //time division 1
 DDRC = 0;
 DDRB=0xFF;
 DDRD=0xFF;
 OCR0A=rSpeed;
 OCR0B=0;
 TCCR0A=0b10100011; // fast PWM, non-inverted
 TCCR0B=0b00000010; // timer prescaler
 
 OCR2A=lSpeed;
 OCR2B=0;
 TCCR2A=0b10100011; // fast PWM, non-inverted
 TCCR2B=0b10100010;
 
 ADCSRA |= (1<<ADEN);
}

uint16_t ADCRead(const int channel) 
{   
 ADMUX = 0b01000000;
 ADMUX |= channel;
 ADCSRA |= (1<<ADSC) | (1<<ADIF);
 while ( (ADCSRA & (1<<ADIF)) == 0);
 ADCSRA &= ~(1<<ADSC);
 return ADC;
}