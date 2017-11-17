/*	tiny0_1.c - 11-13-2017
*	Name & E-mail:  - Joshua Liu jliu056@ucr.edu
*	CS Login: jliu056
*	Lab Section: 21
*	Tiny Follower Project 0.1
*
*
*	I acknowledge all content contained herein, excluding template or example
*	code, is my own original work.
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/portpins.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

//FreeRTOS include files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "usart_ATmega1284.h"

enum MotorState {INIT,A} motor_state;

unsigned int speed;
unsigned char signal;
unsigned char block11;
unsigned char block12;
unsigned char block13;
unsigned char block21;
unsigned char block22;


//void set_PWM(unsigned int frequency) {
//
//
//// Keeps track of the currently set frequency
//// Will only update the registers when the frequency
//// changes, plays music uninterrupted.
//static int current_frequency;
//if (frequency != current_frequency) {
//
//if (!frequency) {
//TCCR3B &= 0x08; //stops timer/counter
//}
//else {
//TCCR3B |= 0x03; // resumes/continues timer/counter
//}
//
//// prevents OCR3A from overflowing, using prescaler 64
//// 0.954 is smallest frequency that will not result in overflow
//if (frequency < 0.954) {
//OCR3A = 0xFFFF;
//}
//
//// prevents OCR3A from underflowing, using prescaler 64
//// 31250 is largest frequency that will not result in underflow
//else if (frequency > 31250) {
//OCR3A = 0x0000;
//}
//
//// set OCR3A based on desired frequency
////else
//OCR3A = (short)(8000000 / (128 * frequency)) - 1;
////OCR3A = (short)frequency;
//
//TCNT3 = 0; // resets counter
//current_frequency = frequency;
//}
//}

void PWM_on() {
	TCCR0A |= (1 << COM0A1) | (1 << WGM00);
	TCCR0B |= (1 << CS00) | (1 << CS02);
	OCR0A = 0;
}

void PWM_off() {
	TCCR0A = 0x00;
	TCCR0B = 0x00;
}

void Motor_Init(){
	motor_state = INIT;
}

void Motor_Tick(){
	//Transitions
	switch(motor_state){
		
		case INIT:
		if (!USART_HasReceived(0)) {
			motor_state = INIT;
		}
		else {
			motor_state = A;
		}
		break;
		
		case A:
		motor_state = A;
		break;
		
		default:
		motor_state = INIT;
		break;
		
	}
	//Actions
	switch(motor_state){
		
		case INIT:
		PORTC = 0xFF;
		//PORTA = 0x0C;
		break;
		
		case A:
		OCR0A = 150;
		
		signal = USART_Receive(0);
		USART_Flush(0);
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
		PORTC = block11 | (block12 << 4);
		PORTB = block13;
		
		if (signal == 'X') {
			block21 = USART_Receive(0);
			USART_Flush(0);
		}
		else if (signal == 'Y') {
			block22 = USART_Receive(0);
			USART_Flush(0);
		}
		PORTA = block21 | (block22 << 4);
		break;
		
		
		default:
		break;
		
	}
}

void MotorTask()
{
	Motor_Init();
	for(;;)
	{
		Motor_Tick();
		vTaskDelay(1);
	}
}

void StartSecPulse(unsigned portBASE_TYPE Priority)
{
	xTaskCreate(MotorTask, (signed portCHAR *)"MotorTask", configMINIMAL_STACK_SIZE, NULL, Priority, NULL );
}

int main(void)
{
	DDRA = 0xFF;	PORTA = 0x00;
	DDRB = 0xFF;	PORTB = 0x00;
	DDRC = 0xFF;	PORTC = 0x00;
	
	PWM_on();
	
	initUSART(0);
	USART_Flush(0);
	
	//Start Tasks
	StartSecPulse(1);
	//RunSchedular
	vTaskStartScheduler();
	
	return 0;
}