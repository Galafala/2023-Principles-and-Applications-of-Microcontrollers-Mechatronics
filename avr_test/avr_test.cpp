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
    DDRD = 0b10000000;
    while (1)
    {
        PORTD = 0b10000000;
        _delay_ms(1000);
        PORTD = 0b00000000;
        _delay_ms(1000);
    }
    return 0;
}