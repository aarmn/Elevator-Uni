/*
 * ElevatorProject.cpp
 *
 *  Author: Mahyar-Rajaei
 */ 


#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

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

/************* defining Times ***************/
#define TIME_TO_PASS_ONE_FLOOR 1000 //takes 1000ms to leave one floor to another
#define TIME_TO_OPEN_THE_DOOR 2000
#define TIME_TO_CLOSE_THE_DOOR 2000
#define WAIT_TIME_TO_CLOSE_THE_DOOR 1000

enum direction {
		CLOCK_WISE,
		ANTI_CLOCK_WISE
	};

typedef int floor_t;

void lift_motor_request(floor_t current_floor, floor_t dest_floor);
void update_floor(floor_t, direction);
void update_7_seg(floor_t, direction);
void lift_next_floor(floor_t, direction); //moves lift either one floor down or up
void rotate_lift_motor_for_one_floor(direction); 


void open_the_door();
void close_the_door();


int main(void)
{

	//floor_t current_floor = 4;
	//floor_t dest_floor = 2;
	LIFT_MOTOR_DDR |= (1 << LIFT_MOTOR_IN1) | (1 << LIFT_MOTOR_IN2) | (1 << LIFT_MOTOR_EN);
	DOOR_MOTOR_DDR |= (1 << DOOR_MOTOR_IN1) | (1 << DOOR_MOTOR_IN2) | (1 << DOOR_MOTOR_EN);

	lift_motor_request(1, 2);

	open_the_door();
	_delay_ms(WAIT_TIME_TO_CLOSE_THE_DOOR);
	close_the_door();

	while(1) {
		
	}
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
