/*
 * ESC.h
 *
 * Created: 22/02/2014 19:51:06
 *  Author: Hon Bo Xuan
 */ 


#ifndef ESC_H_
#define ESC_H_

/*
Phase 1:	B,A
Phase 2:	C,A
Phase 3:	C,B
Phase 4:	A,B
Phase 5:	A,C
Phase 6:	B,C
*/

/*
Port:
ZCFilter (account for hysteresis

Feature list:
Battery voltage drop compensation
COMP_PWM: Low phase low side FET is PWMed. When off, high side FET is switched on. Can be done in software. Cleaner waveform.
Braking, regenerative
True current sensing
Motor/propeller fault sensing (probably motor and/or load specific)

Speed report via SPI/I2C
*/

#define HIGH_A		PC1
#define HIGH_B		PC2
#define HIGH_C		PC3
#define	HIGH_DDR	DDRC
#define HIGH_PORT	PORTC

#define LOW_A	PCTL0 //PSCOUT00 => PD0
#define LOW_B	PCTL1 //PSCOUT10 => PC0
#define LOW_C	PCTL2 //PSCOUT20 => PB0

#define COMPARATOR_A	AC0O //PD7, ACMP0
#define COMPARATOR_B	AC1O //PC6, ACMP1
#define COMPARATOR_C	AC2O //PD5, ACMP2

//Flags
#define COMPARATOR_STATE	0
#define RUNNING_MODE		1
#define COMMUTATE_FLAG		2
#define DIRECTION			3

#define TOP			1024
#define START_DUTY	1024
#define IDLE_DUTY	256
#define TEST_DUTY	128

#endif /* ESC_H_ */