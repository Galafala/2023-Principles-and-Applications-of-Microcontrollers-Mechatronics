// Define the microcontroller we are using
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

// Include the libraries we are going to use
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

// Write your main code
int main(void)
{
    DDRD = 0b11111111;
    while (1)
    {
        PORTD = 0b00000001;
        _delay_ms(500);
        PORTD = 0b00000000;
        _delay_ms(500);
    }
    return 0;
}