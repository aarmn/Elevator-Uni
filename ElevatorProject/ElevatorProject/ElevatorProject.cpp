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



/************* defining Ports **************/
#define LIFT_MOTOR_DDR DDRC
#define LIFT_MOTOR_PORT PORTC
#define LIFT_MOTOR_IN1 PC1
#define LIFT_MOTOR_IN2 PC2
#define LIFT_MOTOR_EN PC3

#define DOOR_MOTOR_DDR DDRC 
#define DOOR_MOTOR_PORT PORTC
#define DOOR_MOTOR_IN1 PC4
#define DOOR_MOTOR_IN2 PC5
#define DOOR_MOTOR_EN PC6

#define KEY_SET_DDR DDRA
#define KEY_SET_PORT PORTA
#define KEY_SET_I1 PA0
#define KEY_SET_I2 PA1
#define KEY_SET_I3 PA2
#define KEY_SET_I4 PA3

#define INTERRUPT PD2

/************* defining Times ***************/
#define TIME_TO_PASS_ONE_FLOOR 1000 //takes 1000ms to leave one floor to another
#define TIME_TO_OPEN_THE_DOOR 2000
#define TIME_TO_CLOSE_THE_DOOR 2000
#define WAIT_TIME_TO_CLOSE_THE_DOOR 1000

//request code is 4bit long
//#define NO_REQUEST_CODE 0b0000
#define FLOOR_0_REQUEST_CODE 0b0000
#define FLOOR_1_REQUEST_CODE 0b0001
#define FLOOR_2_REQUEST_CODE 0b0010
#define FLOOR_3_REQUEST_CODE 0b0011
#define KEY_PAD_0_REQUEST_CODE 0b0100
#define KEY_PAD_1_REQUEST_CODE 0b0101
#define KEY_PAD_2_REQUEST_CODE 0b0110
#define KEY_PAD_3_REQUEST_CODE 0b0111
#define KEY_PAD_ALARM_REQUEST_CODE 0b1000
#define KEY_PAD_BIT_MSK 0b00001111


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
floor_t current_floor = 0;




struct request_holder{
	//int* priotity  = (int*) malloc(4 * sizeof(int));
	int number_of_avialable_requests = 0;
	bool request_for_floor[4] = {false , false, false, false};

} request_holder;

void set_request_for_floor(floor_t fl){
		request_holder.request_for_floor[fl] = true;
		request_holder.number_of_avialable_requests++;
}

floor_t get_nearest_available_request(floor_t current_floor){
	
	if (request_holder.number_of_avialable_requests <= 0)
		return -1;

	uint8_t below_distance = 0, above_distance = 0;
	floor_t above_floor = -1;
	for (floor_t f = current_floor; f < NUMBER_OF_FLOORS; f++) 
		if (request_holder.request_for_floor[f]) {
			above_floor = f;
			above_distance = f - current_floor;
			break;
		}
	PORTB = above_distance;
	_delay_ms(1000);

	floor_t below_floor = -1;
	for (floor_t f = current_floor - 1; f >= 0; f--)
		if (request_holder.request_for_floor[f]) {
			below_floor = f;
			below_distance = current_floor - f; 
			break;
		}

	PORTB = below_distance;
	_delay_ms(1000);
	
	request_holder.number_of_avialable_requests--;
	if (below_distance > above_distance && above_floor != -1) {
		request_holder.request_for_floor[above_floor] = false;
		return above_floor;
	} else if (below_distance < above_distance && below_floor != -1) {
		request_holder.request_for_floor[below_floor] = false;
		return below_floor;
	} else {
		request_holder.request_for_floor[current_floor] = false;
		return current_floor;
	}
	
}

void lift_motor_request(floor_t current_floor, floor_t dest_floor);
void update_floor(floor_t, direction);
void update_7_seg(floor_t, direction);
void lift_next_floor(floor_t, direction); //moves lift either one floor down or up
void rotate_lift_motor_for_one_floor(direction); 


void open_the_door();
void close_the_door();

void init_ddr();



ISR(INT0_vect) {

	volatile uint8_t request_code = PINA & KEY_PAD_BIT_MSK; 
	//PORTB = request_code;
	switch (request_code) {

		case KEY_PAD_0_REQUEST_CODE:
		case FLOOR_0_REQUEST_CODE:
			set_request_for_floor(0);
		break;
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
		case KEY_PAD_ALARM_REQUEST_CODE:
		//TODO: alarm ?
		break;

	}
	floor_t nearest_floor = get_nearest_available_request(current_floor);
	PORTB = nearest_floor; 
}



int main(void)
{


	init_ddr();
	DDRB = 0XFF;
	while(1) {
		current_floor = 4;
		set_request_for_floor(0);
		//PORTB = get_nearest_available_request(0);
		//PORTB = 10;

	}
}


void init_ddr(){

	LIFT_MOTOR_DDR |= (1 << LIFT_MOTOR_IN1) | (1 << LIFT_MOTOR_IN2) | (1 << LIFT_MOTOR_EN);
	DOOR_MOTOR_DDR |= (1 << DOOR_MOTOR_IN1) | (1 << DOOR_MOTOR_IN2) | (1 << DOOR_MOTOR_EN);

	KEY_SET_DDR &= ~(1 << KEY_SET_I1) & ~(1 << KEY_SET_I2) & ~(1 << KEY_SET_I4);


	GICR |= (1 << INT0);   //Enable External Interrupts INT0 and INT1
	MCUCR=0x08;  //Configure INT0 active low level triggered and INT1 as falling edge
	sei();     // Enable global interrupts by setting global interrupt enable bit in SREG

}



void lift_motor_request(floor_t current_floor, floor_t dest_floor){
	
	state = MOVING;
	int number_of_floors = current_floor - dest_floor;
	direction dir = number_of_floors >= 0 ? ANTI_CLOCK_WISE : CLOCK_WISE; //specifing direction
	number_of_floors = abs(number_of_floors); //specifing number of floors
	for (int i = 0; i < number_of_floors; i++) 
		lift_next_floor(current_floor, dir);
		
	state = STATIONARY;
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
