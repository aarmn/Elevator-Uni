/*
 * ElevatorProject.cpp
 *
 *  Author: Mahyar-Rajaei
 */ 


#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <alloca.h>


/************ lift specification *************/
#define NUMBER_OF_FLOORS 4



/************* defining Motors **************/
#define LIFT_MOTOR_DDR DDRA
#define LIFT_MOTOR_PORT PORTA
#define LIFT_MOTOR_IN1 PA1
#define LIFT_MOTOR_IN2 PA2
#define LIFT_MOTOR_EN PA3

#define DOOR_MOTOR_DDR DDRA 
#define DOOR_MOTOR_PORT PORTA
#define DOOR_MOTOR_IN1 PA4
#define DOOR_MOTOR_IN2 PA5
#define DOOR_MOTOR_EN PA6

#define KEY_SET_DDR DDRD
#define KEY_SET_PORT PORTD
#define KEY_SET_I1 PD0
#define KEY_SET_I2 PD1
#define KEY_SET_I3 PD3

#define INTERRUPT PD2

/************* defining Times ***************/
#define TIME_TO_PASS_ONE_FLOOR 1000 //takes 1000ms to leave one floor to another
#define TIME_TO_OPEN_THE_DOOR 2000
#define TIME_TO_CLOSE_THE_DOOR 2000
#define WAIT_TIME_TO_CLOSE_THE_DOOR 1000

//request code is 4bit long
//#define NO_REQUEST_CODE 0b0000
#define FLOOR_1_REQUEST_CODE 0b0000
#define FLOOR_2_REQUEST_CODE 0b0001
#define FLOOR_3_REQUEST_CODE 0b0010
#define FLOOR_4_REQUEST_CODE 0b0011
#define KEY_PAD_1_REQUEST_CODE 0b0100
#define KEY_PAD_2_REQUEST_CODE 0b0101
#define KEY_PAD_3_REQUEST_CODE 0b0110
#define KEY_PAD_4_REQUEST_CODE 0b0111
#define KEY_PAD_ALARM_REQUEST_CODE 0b1111


enum direction {
		CLOCK_WISE,
		ANTI_CLOCK_WISE
	};

enum State {
		MOVING,
		STATIONARY	
	};


typedef int floor_t;

State state = STATIONARY;
floor_t current_floor = 1;




struct request_holder{
	//int* priotity  = (int*) malloc(4 * sizeof(int));
	uint8_t number_of_avialable_requests = 0;
	bool request_for_floor[4] = {false , false, false, false};

} request_holder;

void set_request_for_floor(floor_t fl){
		request_holder.request_for_floor[fl] = true;
		request_holder.number_of_avialable_requests++;
}

	floor_t get_next_available_request(floor_t current_floor){
		if (request_holder.number_of_avialable_requests == 0)
			return -1;
		uint8_t next_floor_rank = 0;
		current_floor = current_floor - 1; //current_floor_index
		floor_t first_above_floor = NULL, first_below_floor = NULL;
		for (floor_t f = current_floor + 1; f < NUMBER_OF_FLOORS; f++) {
			if (request_holder.request_for_floor[f])
				next_floor_rank++;
			if (first_above_floor == NULL)
				first_above_floor = f + 1; //plus 1, to convert floor index to floor number
		}
				
		for (floor_t f = current_floor - 1; f >= 0 ; f--)  {
			if (request_holder.request_for_floor[f])
				next_floor_rank++;
			if (first_below_floor == NULL)
				first_below_floor = f + 1; //plus 1, to convert floor index to floor number

		}


		request_holder.number_of_avialable_requests--;
		if (next_floor_rank > 0) {
			request_holder.request_for_floor[first_above_floor] = false;
			return first_above_floor;
		} else {
			request_holder.request_for_floor[first_below_floor] = false;
			return first_below_floor;
		}

}

void lift_motor_request(floor_t current_floor, floor_t dest_floor);
void update_floor(floor_t, direction);
void update_7_seg(floor_t, direction);
void lift_next_floor(floor_t, direction); //moves lift either one floor down or up
void rotate_lift_motor_for_one_floor(direction); 


void open_the_door();
void close_the_door();



ISR(INT0_vect) {

	volatile uint8_t floor_request_code = PIND; 
	switch (floor_request_code) {

		case KEY_PAD_1_REQUEST_CODE:
		case FLOOR_1_REQUEST_CODE:
			set_request_for_floor(1);
		break;
		case KEY_PAD_2_REQUEST_CODE:
		case FLOOR_2_REQUEST_CODE:
			set_request_for_floor(2);
		break;
		case KEY_PAD_3_REQUEST_CODE:
		case FLOOR_3_REQUEST_CODE:
			set_request_for_floor(3);
		break;
		case KEY_PAD_4_REQUEST_CODE:
		case FLOOR_4_REQUEST_CODE:
			set_request_for_floor(4);
		break;
		case KEY_PAD_ALARM_REQUEST_CODE:
		//TODO: alarm ?
		break;

	}

	if (state == STATIONARY){
		floor_t next_floor = get_next_available_request(current_floor);
		lift_motor_request(current_floor, next_floor);
	}

}



int main(void)
{


	DDRD |= (0 << PB2);
	DDRB |= (1 << PB0);

	current_floor = 1;
	set_request_for_floor(3);
	set_request_for_floor(2);
	set_request_for_floor(4);

	current_floor = get_next_available_request(current_floor);
	if (current_floor == 2)
		PORTB |= (1 << PB0);


	
	GICR |= (1 << INT0);   //Enable External Interrupts INT0 and INT1
	MCUCR=0x08;  //Configure INT0 active low level triggered and INT1 as falling edge

	sei();     // Enable global interrupts by setting global interrupt enable bit in SREG

	while(1) {
		PORTB |= 0B00000000;
		
	}
}


void init_ddr(){

	LIFT_MOTOR_DDR |= (1 << LIFT_MOTOR_IN1) | (1 << LIFT_MOTOR_IN2) | (1 << LIFT_MOTOR_EN);
	DOOR_MOTOR_DDR |= (1 << DOOR_MOTOR_IN1) | (1 << DOOR_MOTOR_IN2) | (1 << DOOR_MOTOR_EN);

}



void lift_motor_request(floor_t current_floor, floor_t dest_floor){
	
	int number_of_floors = current_floor - dest_floor;
	direction dir = number_of_floors >= 0 ? ANTI_CLOCK_WISE : CLOCK_WISE; //specifing direction
	number_of_floors = abs(number_of_floors); //specifing number of floors
	for (int i = 0; i < number_of_floors; i++) 
		lift_next_floor(current_floor, dir);
		
}



void lift_next_floor(floor_t current_floor, direction dir) {
	rotate_lift_motor_for_one_floor(dir);
	update_floor(current_floor, dir);
	update_7_seg(current_floor, dir);
}


void update_floor(floor_t current_floor, direction dir){
	//TODO: 

	return;
}

void update_7_seg(floor_t current_floor, direction dir){
	//TODO:
	return;
}




void rotate_lift_motor_for_one_floor(direction dir){

	if (dir == CLOCK_WISE) {
		LIFT_MOTOR_PORT |= (1 << LIFT_MOTOR_IN1) | (1 << LIFT_MOTOR_EN); 
	} else if (dir == ANTI_CLOCK_WISE) {
		LIFT_MOTOR_PORT |= (1 << LIFT_MOTOR_IN2) | (1 << LIFT_MOTOR_EN); 
	}
	_delay_ms(TIME_TO_PASS_ONE_FLOOR);
	LIFT_MOTOR_PORT &= (0 << LIFT_MOTOR_EN);
}


void open_the_door(){
	
	//clock_wise
	DOOR_MOTOR_PORT |= (1 << DOOR_MOTOR_IN1) | (1 << DOOR_MOTOR_EN); //enables the motor and specifies the direction of rotation
	_delay_ms(TIME_TO_OPEN_THE_DOOR);
	DOOR_MOTOR_PORT &= (0 << DOOR_MOTOR_EN); //disables the motor
}


void close_the_door(){

	//anti_clock_wise
	DOOR_MOTOR_PORT |= (1 << DOOR_MOTOR_IN2) | (1 << DOOR_MOTOR_EN); //enables the motor and specifies the direction of rotation
	_delay_ms(TIME_TO_CLOSE_THE_DOOR);
	DOOR_MOTOR_PORT &= (0 << DOOR_MOTOR_EN); //disables the motor

}
