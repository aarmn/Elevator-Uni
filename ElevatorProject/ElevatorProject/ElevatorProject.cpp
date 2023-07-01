/*
 * ElevatorProject.cpp
 *
 *  Author: Mahyar-Rajaei
 */ 


#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRB |= (1 << PB0);
    while(1)
    {
		PORTB = 0X1;
		_delay_ms(1000);
		PORTB = 0X0;
		_delay_ms(1000);
    }
}