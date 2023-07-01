/*
 * ElevatorProject.cpp
 *
 * Created: 10/04/1402 04:11:45 ب.ظ
 *  Author: aarmn
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

int current_floor = 1;
int active_request = 0;
int[4] requests_queue = {0,0,0,0};

ISR(INT0_vect)
{
	if (PINB4) {
		add_request(1);
	}
	if (PINB5) {
		add_request(2);
	}
	if (PINB6) {
		add_request(3);
	}
	if (PINB6) {
		add_request(4);
	}
}

void add_request(int floor) { // maybe active req out therefore 3?
	if active_request == floor {
		return;
	}
	if active_request == 0 {
		active_request = floor;
	}
	for (int i=0; i<=4; i++) {
		if (requests_queue[i] == floor) return;
		if (requests_queue[i] == 0) requests_queue[i] = floor; return;
	}
}

void pop_into_active_request(void) {
	int close_i=100, close_val=100,
	for (int i=0; i<=4; i++) {
		if (abs(close_val-current_floor)) 
	}
	active_request = requests_queue[0];
	for (int i=1; i<=4; i++) {
		int[i-1] = int[i];
	}
}

void seven_segment(int status) {
	switch (status)
	{
		case 1:  break;
		case 2:  break;
		case 3:  break;
		case 4:  break;
		case 0:  break; //blink setup
	}
}

int main(void)
{
	DDRB &= ~(1 << PB4); // go to floor 1
	DDRB &= ~(1 << PB5); // go to floor 2
	DDRB &= ~(1 << PB6); // go to floor 3
	DDRB &= ~(1 << PB7); // go to floor 4
	DDRC &= ~(1 << PC0); // request floor 1
	DDRC &= ~(1 << PC1); // request floor 2
	DDRC &= ~(1 << PC2); // request floor 3
	DDRC &= ~(1 << PC3); // request floor 4
	DDRC &= ~(1 << PC4); // IR Sensor
	DDRC &= ~(1 << PC5); // alarm key
	DDRA |= (1 << PA0); //
	
	// make D0 D1 D6 C6 pins of 7seg
	
	// PORTD |= (1 << PD2); // Enable pull-up resistor on buttons above
	
	DDRD &= ~(1 << PD2); // Set PD2 as input
	PORTD |= (1 << PD2); // Enable pull-up resistor on PD2
	MCUCR |= (1 << ISC01); // Set for falling edge detection
	GICR |= (1 << INT0); // Enable external interrupt INT0
	
	sei(); // Enable global interrupts
	
	// keys interrupt
	
	
	
    while(1)
    {
        
    }
}