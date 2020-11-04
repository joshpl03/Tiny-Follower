/*
* tiny.c
*
* Created: 11/20/2017 10:02:15 AM
* Author : Joshua Liu
*/

// include necessary header files for microcontroller
#include <avr/io.h>
#include <util/delay.h>
#include "usart_ATmega1284.h"

// construct finite state machine for USART receiving
enum USARTState {UINIT,Recieving} usart_state;

// construct finite state machine for robot movement
enum MoveState {MINIT,Move} move_state;

// USART variables
unsigned char signal;
unsigned char block11, block12, block13, block21, block22;

// Move variables
unsigned char rec_flag;
unsigned int hpos, rad;

// Two sets of PID variables
double kp1, ki1, kd1, kp2, ki2, kd2;
// Horizontal position and radius equilibrium values, state numerical counter
unsigned int hposT, radT, timepassed;
// PID calculation helper variables
int err1, errSum1, lastErr1, derr1, adjust1, err2, errSum2, lastErr2, derr2, adjust2;
unsigned int hpos_last, rad_last;

// Initialize PWM
void PWM_on() {
	TCCR3A |= (1 << COM3A1) | (1 << WGM30);
	TCCR1A |= (1 << COM1A1) | (1 << WGM10);
	// COM3A0: Toggle PB6 on compare match between counter and OCR3A
	TCCR3B |= (1 << CS30) | (1 << CS32);
	TCCR1B |= (1 << CS10) | (1 << CS12);
	// WGM32: When counter (TCNT3) matches OCR3A, reset counter
	// CS31 & CS30: Set a prescaler of 64
	OCR3A = 0;
	OCR1A = 0;
}

// PWM 1 to control passenger-side wheel speed
void set_PWM1(int value) {
	if (value < 20) {
		OCR3A = 0;
	}
	else {
		OCR3A = value;
	}
}

// PWM 2 to control driver-side wheel speed
void set_PWM2(int value) {
	if (value < 20) {
		OCR1A = 0;
	}
	else {
		OCR1A = value;
	}
}

// Stop both PWMs to remain in place
void PWM_off() {
	TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCCR1A = 0x00;
	TCCR1B = 0x00;
}

// Define USART finite state machine
void USART_Tick(){
	//Transitions
	switch(usart_state){
			
		// Remains in initial state until receiving data
		case UINIT:
		if (!USART_HasReceived(0)) {
			usart_state = UINIT;
		}
		else {
			usart_state = Recieving;
		}
		break;
		
		// Continually loop this state once entered
		case Recieving:
		usart_state = Recieving;
		break;
		
		// Default state
		default:
		usart_state = UINIT;
		break;
		
	}
	//Actions
	switch(usart_state){
		
		// Set flag to indicate no USART data received
		case UINIT:
		rec_flag = 0;
		break;
		
		// Save previous received data
		// set flag to indicate USART data received
		// receive new USART data (horizontal position and radius of the ball)
		case Recieving:
		hpos_last = hpos;
		rad_last= rad;
		rec_flag = 1;
		signal = USART_Receive(0);
		USART_Flush(0);
			
		// Receive horizontal position data blocks A,B,C,D (4 blocks)
		if (signal == 'A') {
			block11 = USART_Receive(0);
			USART_Flush(0);
		}
		else if (signal == 'B') {
			block12 = USART_Receive(0);
			USART_Flush(0);
		}
		else if (signal == 'C') {
			block13 = USART_Receive(0);
			USART_Flush(0);
		}
		hpos = block11 + (block12 << 4);
		hpos += block13 << 8;
			
		// Receive radius data blocks X,Y (2 blocks)
		if (signal == 'X') {
			block21 = USART_Receive(0);
			USART_Flush(0);
		}
		else if (signal == 'Y') {
			block22 = USART_Receive(0);
			USART_Flush(0);
		}
		rad = block21 | (block22 << 4);
		break;
		
		// No actions in default state
		default:
		break;
		
	}
}

// Define robot movement finite state machine
void Move_Tick(){
	//Transitions
	switch(move_state){
		
		// Remain in initial state until receiving data
		case MINIT:
		if (rec_flag) {
			move_state = Move;
			//PORTC = 0xF0;
		}
		else {
			move_state = MINIT;
			//PORTC = 0x0F;
		}
		break;
		
		// Continually loop this once entered
		case Move:
		move_state = Move;
		break;
		
		// Default state
		default:
		move_state = MINIT;
		break;
		
	}
	//Actions
	switch(move_state){
		
		// Set intial values for all variables
		case MINIT:
		hposT = 320; // midpoint for horizontal position of ball
		hpos = hposT;
		radT = 70; // desired size of ball
		rad = radT;
		kp1 = 2;
		ki1 = 0;
		kd1 = 0;
		errSum1 = 0;
		timepassed = 1;
		lastErr1 = 0;
		kp2 = 0.2;
		ki2 = 0;
		kd2 = 0;
		errSum2 = 0;
		lastErr2 = 0;
		break;
		
		// Calculate motor speed values using PID control and apply
		case Move:
			
		// Forward/backward movement PID control calculations
		err1 = radT - rad;
		errSum1 += err1 * timepassed;
		derr1 = (err1 - lastErr1) / timepassed;
		adjust1 = kp1 * err1 + ki1 * errSum1 + kd1 * derr1;
			
		// Left/right PID control calculations
		err2 = hpos - hposT;
		errSum2 += err2 * timepassed;
		derr2 = (err2 - lastErr2) / timepassed;
		adjust2 = kp2 * err2 + ki2 * errSum2 + kd2 * derr2;
		
		// Activate and set PWMs to adjust distance with a left-turning bias
		if (err1 > 0) {
			PORTB &= 0xF0;
			PORTB |= 0x05;
			set_PWM1(adjust1+adjust2);
			set_PWM2(adjust1-adjust2);
			//OCR3A = adjust1;
			//OCR1A = adjust1;
		}
		// Activate and set PWMs to adjust distance with a right-turning bias
		else if (err1 < 0) {
			PORTB &= 0xF0;
			PORTB |= 0x0A;
			set_PWM1(-adjust1-adjust2);
			set_PWM2(-adjust1+adjust2);
			//OCR3A = -adjust1;
			//OCR1A = -adjust1;
		}
		// Activate and set PWMs to adjust distance when ball is already centered
		// Remain in place if the ball is stationary
		else {
			PORTB &= 0xF0;
			if (err2 > 0) {
				PORTB |= 0x09;
				set_PWM1(adjust2);
				set_PWM2(adjust2);
				//OCR3A = adjust2;
				//OCR1A = adjust2;
			}
			else if (err2 < 0) {
				PORTB |= 0x06;
				set_PWM1(-adjust2);
				set_PWM2(-adjust2);
				//OCR3A = -adjust2;
				//OCR1A = -adjust2;
			}
			else {
				set_PWM1(0);
				set_PWM2(0);
			}
		}
		// Save previous error data for PID
		lastErr1 = err1;
		lastErr2 = err2;
		break;
		
		// Default state
		default:
		break;
		
	}
}

int main(void)
{
	// Initialize microcontroller registers
	DDRA = 0xFF;	PORTA = 0x00;
	DDRB = 0xFF;	PORTB = 0x00;
	DDRC = 0xFF;	PORTC = 0x00;
	DDRD = 0xFF;	PORTD = 0x00;
	
	// Initialize PWM
	PWM_on();
	
	// Initialize USART communication
	initUSART(0);
	USART_Flush(0);
	
	// Set finite state machine initial states
	usart_state = UINIT;
	move_state = MINIT;
	
	// Initialize finite state machines
	while (1) {
		USART_Tick();
		Move_Tick();
	}
}

