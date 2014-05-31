
/*
 * Assembly.s
 *
 * Created: 24/02/2014 23:22:21
 *  Author: Hon Bo Xuan
 */ 

#include <avr/io.h>
#include "ESC.h"

#define	ZERO			r1
#define	ONE				r2
#define	COMM_CNT		r3
#define CYCLE_CNT		r4
#define	TCNT0_TMP		r5
#define	SAMPLE_CNT		r6

#define SREG_TEMP			r16
#define ISR_RMP				r17
#define	RMP					r18
#define	FLAGS				r19
#define	COMPARATOR_MASK		r20
#define	SAMPLE_SUM			r21
#define	DUTY_H				r22
#define	DUTY_L				r23
#define	COMP_DUTY_H			r24
#define	COMP_DUTY_L			r25

.global Initialise
.global Loop
.global TIMER0_COMPB_vect
.global	WDT_vect
.global INT1_vect
.global SPI_STC_vect

/*==========Routines==========*/
/*----------Initialise----------*/
Initialise:
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO

	ldi		RMP, 1
	mov		ONE, RMP
	clr		FLAGS
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	
	sbr		FLAGS, (1 << DIRECTION)
	sbis	_SFR_IO_ADDR(DIRECTION_PORT), DIRECTION_PIN
	cbr		FLAGS, (1 << DIRECTION)

	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)
	clr		ZH
	ldi		ZL, pm_lo8(Phase1)

	sei
	ret

/*----------Disarm----------*/
Disarm:
	cli

	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
	
	ldi		DUTY_H, 0
	ldi		DUTY_L, 0
	ldi		COMP_DUTY_H, (TOP >> 8)
	ldi		COMP_DUTY_L, (TOP & 255)
	sts		OCR0SBH, COMP_DUTY_H
	sts		OCR0SBL, COMP_DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L
	
	cbr		FLAGS, (1 << COMMUTATE_FLAG)
	cbr		FLAGS, (1 << RUNNING_MODE)
	cbr		FLAGS, (1 << ARMED)

	sei

	rjmp	Loop

/*----------Commutate----------*/
Commutate:
	wdr
	sts		TCNT0, ZERO
	
	sts		OCR0SBH, COMP_DUTY_H
	sts		OCR0SBL, COMP_DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L
	
	inc		COMM_CNT

	ijmp
Phase1: ; B,A
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
	sts		LOW_A, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_B
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_C)
	
	
	inc		CYCLE_CNT ; For control input timeout

	
	sbrs	FLAGS, DIRECTION
	rjmp	Phase1Dir0
Phase1Dir1:
	ldi		ZH, pm_hi8(Phase6) ; Unneccessary
	ldi		ZL, pm_lo8(Phase6)
	rjmp	CommutateEnd
Phase1Dir0:
	ldi		ZH, pm_hi8(Phase2) ; Unneccessary
	ldi		ZL, pm_lo8(Phase2)
	rjmp	CommutateEnd
Phase2: ; C,A
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
	sts		LOW_A, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_C
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_B)
	sbrs	FLAGS, DIRECTION
	rjmp	Phase2Dir0
Phase2Dir1:
	ldi		ZH, pm_hi8(Phase1) ; Unneccessary
	ldi		ZL, pm_lo8(Phase1)
	rjmp	CommutateEnd
Phase2Dir0:
	ldi		ZH, pm_hi8(Phase3) ; Unneccessary
	ldi		ZL, pm_lo8(Phase3)
	rjmp	CommutateEnd
Phase3: ; C,B
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_C, ZERO
	sts		LOW_B, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_C
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)
	sbrs	FLAGS, DIRECTION
	rjmp	Phase3Dir0
Phase3Dir1:
	ldi		ZH, pm_hi8(Phase2) ; Unneccessary
	ldi		ZL, pm_lo8(Phase2)
	rjmp	CommutateEnd
Phase3Dir0:
	ldi		ZH, pm_hi8(Phase4) ; Unneccessary
	ldi		ZL, pm_lo8(Phase4)
	rjmp	CommutateEnd
Phase4: ; A,B
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_C, ZERO
	sts		LOW_B, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_A
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_C)
	sbrs	FLAGS, DIRECTION
	rjmp	Phase4Dir0
Phase4Dir1:
	ldi		ZH, pm_hi8(Phase3) ; Unneccessary
	ldi		ZL, pm_lo8(Phase3)
	rjmp	CommutateEnd
Phase4Dir0:
	ldi		ZH, pm_hi8(Phase5) ; Unneccessary
	ldi		ZL, pm_lo8(Phase5)
	rjmp	CommutateEnd
Phase5: ; A,C
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_A
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_B)
	sbrs	FLAGS, DIRECTION
	rjmp	Phase5Dir0
Phase5Dir1:
	ldi		ZH, pm_hi8(Phase4) ; Unneccessary
	ldi		ZL, pm_lo8(Phase4)
	rjmp	CommutateEnd
Phase5Dir0:
	ldi		ZH, pm_hi8(Phase6) ; Unneccessary
	ldi		ZL, pm_lo8(Phase6)
	rjmp	CommutateEnd
Phase6: ; B,C
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_B
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)
	sbrs	FLAGS, DIRECTION
	rjmp	Phase6Dir0
Phase6Dir1:
	ldi		ZH, pm_hi8(Phase5) ; Unneccessary
	ldi		ZL, pm_lo8(Phase5)
	rjmp	CommutateEnd
Phase6Dir0:
	ldi		ZH, pm_hi8(Phase1) ; Unneccessary
	ldi		ZL, pm_lo8(Phase1)
	rjmp	CommutateEnd
CommutateEnd:
	sbrc	FLAGS, RUNNING_MODE
	reti
	rjmp	Loop

/*----------ZC Filter----------*/
//Startup Mode
//Full power startup has no PWM noise
StartupModeZCFilter:
	clr		SAMPLE_CNT
	sbrs	FLAGS, COMPARATOR_STATE
	rjmp	StartupModeComparatorStateLow
StartupModeComparatorStateHigh:
	cpi		SAMPLE_SUM, 1

	brsh	ZCFilterEnd
	lds		TCNT0_TMP, TCNT0
	cbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
StartupModeComparatorStateLow:
	cpi		SAMPLE_SUM, 32

	brlo	ZCFilterEnd 
	lds		TCNT0_TMP, TCNT0
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
//Running Mode
RunningModeZCFilter:
	clr		SAMPLE_CNT
	sbrs	FLAGS, COMPARATOR_STATE
	rjmp	RunningModeComparatorStateLow
RunningModeComparatorStateHigh:
	cpi		SAMPLE_SUM, 16
	brsh	ZCFilterEnd
	lds		TCNT0_TMP, TCNT0
	cbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
RunningModeComparatorStateLow:
	cpi		SAMPLE_SUM, 120
	brlo	ZCFilterEnd 
	lds		TCNT0_TMP, TCNT0
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
//End of ZC Filter
ZCFilterEnd:
	clr		SAMPLE_SUM
	rjmp	Loop

/*----------ZC Event----------*/
ZCEvent:
RunningMode:
	sbrs	FLAGS, RUNNING_MODE
	rjmp	StartupMode

	sbrc	CYCLE_CNT, 7 ; 128
	rjmp	Disarm ; N cycles without control input, timeout

	sts		TCNT0, ZERO
	asr		TCNT0_TMP ; Divide by 2
	sts		OCR0B, TCNT0_TMP
	sbr		FLAGS, (1 << COMMUTATE_FLAG)

	rjmp	Loop
StartupMode:
	;inc		COMM_CNT ; Shifted to actual commutation routine
	;sbrs	COMM_CNT, 4 ; 16
	sbrs	COMM_CNT, 5 ; 32
	;sbrs	COMM_CNT, 6 ; 64
	rjmp	Commutate
RunningModeTransition:
	; Idle duty cycle
	ldi		DUTY_H, (IDLE_DUTY >> 8)
	ldi		DUTY_L, (IDLE_DUTY & 255)
	ldi		COMP_DUTY_H, ((TOP-IDLE_DUTY) >> 8)
	ldi		COMP_DUTY_L, ((TOP-IDLE_DUTY) & 255)

	clr		CYCLE_CNT ; Using CYCLE_CNT for control input timeout

	sbr		FLAGS, (1 << RUNNING_MODE)
	rjmp	RunningMode



/*==========Main Loop Start==========*/
Loop:

/*----------Arm/Disarm Checker----------*/
ArmedChecker:
	sbrc	FLAGS, ARMED
	rjmp	ArmedCheckerEnd

	//Beep

	//Check for arming signal
	//Do nothing else, except service interrupts

	rjmp	Loop
ArmedCheckerEnd:

/*----------Commutation Timeout Checker----------*/
//Indicates fault
//Should disarm
//Currently goes back to startup mode
CommutationTimeoutChecker:
	sbrs	FLAGS, COMM_TIMEOUT_FLAG
	rjmp	CommutationTimeoutCheckerEnd

StartupModeTransition:
	; Prepare for clean startup
	cbr		FLAGS, (1 << COMM_TIMEOUT_FLAG)
	cbr		FLAGS, (1 << COMMUTATE_FLAG)
	cbr		FLAGS, (1 << RUNNING_MODE)
	clr		COMM_CNT

	; Startup duty cycle
	ldi		DUTY_H, (START_DUTY >> 8)
	ldi		DUTY_L, (START_DUTY & 255)
	ldi		COMP_DUTY_H, ((TOP-START_DUTY) >> 8)
	ldi		COMP_DUTY_L, ((TOP-START_DUTY) & 255)
	
	rjmp	Commutate
CommutationTimeoutCheckerEnd:

/*----------Comparator Sampler----------*/
ComparatorSampler:
	lds		RMP, ACSR
	and		RMP, COMPARATOR_MASK
	breq	ComparatorSampleLow
ComparatorSampleHigh:
	inc		SAMPLE_SUM
ComparatorSampleLow:
	inc		SAMPLE_CNT
	sbrs	FLAGS, RUNNING_MODE
	rjmp	StartupModeSensitivity
RunningModeSensitivity:
	;sbrs	SAMPLE_CNT, 4 ; 16
	;sbrs	SAMPLE_CNT, 5 ; 32
	;sbrs	SAMPLE_CNT, 6 ; 64
	sbrs	SAMPLE_CNT, 7 ; 128
	rjmp	Loop
	rjmp	RunningModeZCFilter
StartupModeSensitivity:
	;sbrs	SAMPLE_CNT, 4 ; 16
	sbrs	SAMPLE_CNT, 5 ; 32 (use if full power start with 32,1)
	;sbrs	SAMPLE_CNT, 6 ; 64
	;sbrs	SAMPLE_CNT, 7 ; 128
	rjmp	Loop
	rjmp	StartupModeZCFilter


	
/*==========Interrupt Handlers==========*/
/*----------Running Mode Commutation Timer----------*/
TIMER0_COMPB_vect:
	sbrs	FLAGS, COMMUTATE_FLAG
	reti
	cbr		FLAGS, (1 << COMMUTATE_FLAG)
	rjmp	Commutate

/*----------Commutation Timeout Timer----------*/
WDT_vect:
	sbr		FLAGS, (1 << COMM_TIMEOUT_FLAG)
	reti

/*----------Direction Control Pin----------*/
INT1_vect:
	; Check INT1 pin (PB2), manage FLAGS, (1 << DIRECTION)
	sbr		FLAGS, (1 << DIRECTION)
	sbis	_SFR_IO_ADDR(DIRECTION_PORT), DIRECTION_PIN
	cbr		FLAGS, (1 << DIRECTION)
	reti

/*----------SPI Control Input----------*/
SPI_STC_vect:





	sbr		FLAGS, (1 << ARMED) ; Any input via SPI arms ESC (for now)





	sbrs	FLAGS, RUNNING_MODE
	reti

	lds		ISR_RMP, SPDR
	in		SREG_TEMP, _SFR_IO_ADDR(SREG) ; Not always necessary

	sbrc	FLAGS, CTRL_DELIM_RXED
	rjmp	RXDuty
RXDelim:
	cpi		ISR_RMP, CTRL_DELIM
	brne	SPI_STC_vect_end
	sbr		FLAGS, (1 << CTRL_DELIM_RXED)
	rjmp	SPI_STC_vect_end
RXDuty:
	sbrc	FLAGS, CTRL_DUTY_BYTE
	rjmp	RXDutyLow
RXDutyHigh:
	mov		DUTY_H, ISR_RMP
	sbr		FLAGS, (1 << CTRL_DUTY_BYTE)
	rjmp	SPI_STC_vect_end
RXDutyLow:
	mov		DUTY_L, ISR_RMP
	cbr		FLAGS, (1 << CTRL_DELIM_RXED)
	cbr		FLAGS, (1 << CTRL_DUTY_BYTE)
DutyCycleUpdate:
	ldi		COMP_DUTY_H, (TOP >> 8)
	ldi		COMP_DUTY_L, (TOP & 255)
	sub		COMP_DUTY_L, DUTY_L
	sbc		COMP_DUTY_H, DUTY_H

	clr		CYCLE_CNT ; Using CYCLE_CNT for control input timeout

SPI_STC_vect_end:
	; Next byte for SPI master (speed)
	sts		SPDR, ISR_RMP

	out		_SFR_IO_ADDR(SREG), SREG_TEMP ; Not always necessary
	reti