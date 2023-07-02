/*
 * ElevatorProject.cpp
 *
 * Created: 10/04/1402 04:11:45 ب.ظ
 *  Author: aarmn
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

int current_floor = 1;
int active_request = 0;
int requests_queue[4] = {0,0,0,0};
	
void add_request(int floor) { // maybe active req out therefore 3?
	if (active_request == floor) {
		return;
	}
	if (active_request == 0) {
		active_request = floor;
	}
	for (int i=0; i<=4; i++) {
		if (requests_queue[i] == floor) return;
		if (requests_queue[i] == 0) requests_queue[i] = floor; return;
	}
}

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

uint16_t adc_read(uint8_t channel) {
	// ADMUX &= 0xF0;             // Clear the older channel that was read
	// ADMUX |= channel;          // Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);         // Starts a new conversion
	while(ADCSRA & (1<<ADSC));   // Wait until the conversion is done
	return ADCW;                 // Returns the ADC value of the chosen channel
}

void pop_into_active_request(void) {
	int close_i=100, close_val=100;
	for (int i=0; i<=4; i++) {
	//	if (abs((close_val-current_floor))) 
	}
	active_request = requests_queue[0];
	for (int i=1; i<=4; i++) {
		requests_queue[i-1] = requests_queue[i];
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
	DDRB |= (1 << PB4); // 7seg 0
	DDRB |= (1 << PB5); // 7seg 1
	DDRB |= (1 << PB6); // 7seg 2
	DDRB |= (1 << PB7); // 7seg 3
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
	
	// adc init
    ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));   // 16Mhz/128 = 125Khz the ADC reference clock
    ADMUX |= (1<<REFS0)|(1<<REFS1);                            // Voltage reference from Avcc (5v)
	//ADMUX |= (1<<ADLAR);
	ADMUX &= 0xFF;
    ADCSRA |= (1<<ADEN);                            // Turn on ADC
    ADCSRA |= (1<<ADATE);                            // Do an initial conversion because this one is the slowest and to ensure that everything is up and running
	
	sei(); // Enable global interrupts
	
	// keys interrupt
	
	uint16_t a;
	int b;
	
    while(1)
    {
		// **** TEST BCD 7SEG and ADC ****
		//
		//int b = ((a/100) << 4);
		a = adc_read(0);// /100; // can be trouble some in 1024
		// b = (a<<4);
		//b = 5;
		if (a > 700) {
			b = 15;
		}
		else {
			b = 1;
		}
		PORTB = b << 4;
		_delay_ms(1000);
		
		// ^^^^ TEST BCD 7SEG and ADC ^^^^
    }
}