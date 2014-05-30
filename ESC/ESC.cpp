/*
 * ESC.cpp
 *
 * Created: 22/02/2014 19:50:06
 *  Author: Hon Bo Xuan
 */ 

#define F_CPU 16000000UL

//#include <avr/interrupt.h>
#include <avr/io.h>
//#include <stdint.h>

#include "ESC.h"

extern "C" {
	void Initialise(void);
	void Loop(void);
}

int main(void) {
	MCUCR |= (1 << PUD); //Disable internal pull-ups
	HIGH_DDR |= (1 << HIGH_A)|(1 << HIGH_B)|(1 << HIGH_C); //High side control pins
	
	
	
	
	//DDRB |= (1 << PB7)|(1 << PB6); //Debugging
	
	
	
	
	
	
	//Control via PWM Input Capture Timer
	//TCCR1B |= (1 << ICES1)|(1 << CS10); //Rising edge trigger, no prescaler
	//TIMSK1 |= (1 << ICIE1); //Enable Input Capture Interrupt
	
	
	
	
	
	//Control via SPI
	DDRD |= (1 << PD2); //MISO
	DDRD &= ~((1 << PD0)|(1 << PD3)|(1 << PD4)); //SS, MOSI, SCK
	MCUCR |= (1 << SPIPS); //Redirect SPI to alternate pins
	SPCR |= (1 << SPIE)|(1 << SPE); //Interrupt enable, SPI enable
	
	
	
	//Running Mode Timer
	//TIMSK0 |= (1 << OCIE0B)|(1 << TOIE0);
	TIMSK0 |= (1 << OCIE0B);
	OCR0B = 0;
	TCCR0B |= (1 << CS00); //No prescaler
	//TCCR0B |= (1 << CS01); //1/8 prescaler
	//TCCR0B |= (1 << CS01)|(1 << CS00); //1/64 prescaler
	
	//Watchdog Timer
	WDTCSR |= (1 << WDIE); //Interrupt mode
	
	//Direction Change Interrupt
	EICRA |= (1 << ISC10); //Any logical change
	EIMSK |= (1 << INT1); //Enable INT1
	
	//Power Stage Controller Configuration
	PLLCSR |= (1 << PLLF)|(1 << PLLE); //Enable PLL, 64MHz
	//PLLCSR |= (1 << PLLE); //Enable PLL, 32MHz
	
	
	
	
	
	
	
	
	
	//Power Stage Controller 0
	/*
	//Original
	DDRD |= (1 << PD0);
	PSOC0 |= (1 << POEN0A); //PSC0OUTA enable
	OCR0SA = 0; //Deadtime
	OCR0RB = TOP;
	PCNF0 = (1 << POP0)|(1 << PCLKSEL0);
	*/
	//PCNF0 = (1 << POP0);
	
	
	
	
	
	//Power Stage Controller 0
	//Testing using PSCOUT01 to free PD0 (using PB7)
	DDRB |= (1 << PB7);
	PSOC0 |= (1 << POEN0B); //PSC0OUTB enable
	OCR0SA = 0;
	OCR0RA = 0; //Deadtime
	//OCR0SB = 0; //Controls duty cycle (TOP-desired)
	OCR0RB = TOP;
	PCNF0 = (1 << POP0)|(1 << PCLKSEL0);
	//PCNF0 = (1 << POP0);
	
	
	
	
	
	
	
	
	//Power Stage Controller 1
	DDRC |= (1 << PC0);
	PSOC1 |= (1 << POEN1A); //PSC1OUTA enable
	OCR1SA = 0; //Deadtime
	OCR1RB = TOP;
	PCNF1 = (1 << POP1)|(1 << PCLKSEL1);
	//PCNF1 = (1 << POP1);
	
	//Power Stage Controller 2
	DDRB |= (1 << PB0);
	PSOC2 |= (1 << POEN2A); //PSC2OUTA enable
	OCR2SA = 0; //Deadtime
	OCR2RB = TOP;
	PCNF2 = (1 << POP2)|(1 << PCLKSEL2);
	//PCNF2 = (1 << POP2);
	
	//Comparators
	DDRC &= ~(1 << PC6);
	DDRD &= ~((1 << PD5)|(1 << PD6)|(1 << PD7));
	DIDR0 |= (1 << ADC3D)|(1 << ADC2D); //Disable digital input on ACMPM and ACMP2
	DIDR1 |= (1 << ACMP0D)|(1 << ADC10D); //Disable digital input on ACMP0 and ACMP1
	AC0CON |= (1 << AC0EN)|(1 << AC0M2); //Enable AC0, use ACMPM
	AC1CON |= (1 << AC1EN)|(1 << AC1M2); //Enable AC1, use ACMPM
	AC2CON |= (1 << AC2EN)|(1 << AC2M2); //Enable AC2, use ACMPM
	
	//Assembly
	Initialise();
	Loop();
}