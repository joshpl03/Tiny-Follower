/*
 * tiny.c
 *
 * Created: 11/20/2017 10:02:15 AM
 * Author : Joshua Liu
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "usart_ATmega1284.h"

enum USARTState {UINIT,Recieving} usart_state;
enum MoveState {MINIT,Move} move_state;

unsigned int speed;
// USART variables
unsigned char signal;
unsigned char block11, block12, block13, block21, block22;
// Move variables
unsigned char rec_flag;
unsigned int hpos, rad;
// PID variables
double kp1, ki1, kd1, kp2, ki2, kd2;
unsigned int hposT, radT, timepassed;
int err1, errSum1, lastErr1, derr1, adjust1, err2, errSum2, lastErr2, derr2, adjust2;

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

void set_PWM1(int value) {
	if (value < 20) {
		OCR3A = 0;
	}
	else {
		OCR3A = value;
	}
}

void set_PWM2(int value) {
	if (value < 20) {
		OCR1A = 0;
	}
	else {
		OCR1A = value;
	}
}

void PWM_off() {
	TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCCR1A = 0x00;
	TCCR1B = 0x00;
}

void USART_Tick(){
	//Transitions
	switch(usart_state){
		
		case UINIT:
		if (!USART_HasReceived(0)) {
			usart_state = UINIT;
		}
		else {
			usart_state = Recieving;
		}
		break;
		
		case Recieving:
		usart_state = Recieving;
		break;
		
		default:
		usart_state = UINIT;
		break;
		
	}
	//Actions
	switch(usart_state){
		
		case UINIT:
		rec_flag = 0;
		break;
		
		case Recieving:
		
		signal = USART_Receive(0);
		USART_Flush(0);
		if (signal == 'A') {
			block11 = USART_Receive(0);
			USART_Flush(0);
			rec_flag = 1;
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
		//PORTC = hpos;
		hpos += block13 << 8;
		//PORTB = (PORTB & 0xFC) | block13;
		
		if (signal == 'X') {
			block21 = USART_Receive(0);
			USART_Flush(0);
		}
		else if (signal == 'Y') {
			block22 = USART_Receive(0);
			USART_Flush(0);
		}
		rad = block21 | (block22 << 4);
		//PORTA = rad;
		break;
		
		
		default:
		break;
		
	}
}

void Move_Tick(){
	//Transitions
	switch(move_state){
		
		case MINIT:
		if (rec_flag) {
			move_state = Move;
		}
		else {
			move_state = MINIT;
		}
		break;
		
		case Move:
		move_state = Move;
		break;
		
		default:
		break;
		
	}
	//Actions
	switch(move_state){
		
		case MINIT:
		hposT = 320;
		hpos = hposT;
		radT = 70;
		rad = radT;
		kp1 = 2;
		ki1 = 0;
		kd1 = 0;
		errSum1 = 0;
		timepassed = 1;
		lastErr1 = 0;
		kp2 = 0.5;
		ki2 = 0;
		kd2 = 0;
		errSum2 = 0;
		lastErr2 = 0;
		break;
		
		case Move:
		// Forward/backward PID control
		err1 = radT - rad;
		//PORTC = err1;
		errSum1 += err1 * timepassed;
		derr1 = (err1 - lastErr1) / timepassed;
		adjust1 = kp1 * err1 + ki1 * errSum1 + kd1 * derr1;
		// Left/right PID control
		err2 = hpos - hposT;
		//PORTC = err2;
		errSum2 += err2 * timepassed;
		derr2 = (err2 - lastErr2) / timepassed;
		adjust2 = kp2 * err2 + ki2 * errSum2 + kd2 * derr2;
		if (err1 > 0) {
			PORTB &= 0xF0;
			PORTB |= 0x05;
			set_PWM1(adjust1+adjust2);
			set_PWM2(adjust1-adjust2);
			//OCR3A = adjust1;
			//OCR1A = adjust1;
		}
		else if (err1 < 0) {
			PORTB &= 0xF0;
			PORTB |= 0x0A;
			set_PWM1(-adjust1-adjust2);
			set_PWM2(-adjust1+adjust2);
			//OCR3A = -adjust1;
			//OCR1A = -adjust1;
		}
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
				OCR3A = 0;
				OCR1A = 0;
			}
		}
		PORTC = adjust1;
		break;
		
		default:
		break;
		
	}
}

int main(void)
{
	DDRA = 0xFF;	PORTA = 0x00;
	DDRB = 0xFF;	PORTB = 0x00;
	DDRC = 0xFF;	PORTC = 0x00;
	DDRD = 0xFF;	PORTD = 0x00;
	
	PWM_on();
	
	initUSART(0);
	USART_Flush(0);
	
    while (1) {
		USART_Tick();
		Move_Tick();
	}
}

