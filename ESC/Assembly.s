
/*
 * Assembly.s
 *
 * Created: 24/02/2014 23:22:21
 *  Author: Hon Bo Xuan
 */ 

#include <avr/io.h>
#include "ESC.h"

#define	ZERO		r1
#define	ONE			r2
#define	COMM_CNT	r3
#define	TCNT0_TMP	r4
#define	TIMEOUT_CNT	r5

#define SREG_TEMP			r16
#define ISR_RMP				r17
#define	RMP					r18
#define	FLAGS				r19
#define	COMPARATOR_MASK		r20
#define	SAMPLE_SUM			r21
#define	SAMPLE_CNT			r22
#define	DUTY_H				r23
#define	DUTY_L				r24
#define	COMP_DUTY_H			r25
#define	COMP_DUTY_L			r26

.global Initialise
.global Loop
.global TIMER0_COMPB_vect
.global	WDT_vect
.global INT1_vect
.global SPI_STC_vect

//Initialisation
Initialise:
	ldi		RMP, 1
	mov		ONE, RMP
	clr		FLAGS
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	sbr		FLAGS, (1 << DIRECTION)
	sbis	_SFR_IO_ADDR(PINB), PB2
	cbr		FLAGS, (1 << DIRECTION)
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)
	clr		ZH
	ldi		ZL, pm_lo8(Phase1)
	;clr		DUTY_H
	;clr		DUTY_L
	;clr		COMP_DUTY_H
	;clr		COMP_DUTY_L
	sei
/*
AllOff:
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
*/
	ret

//Commutation
Commutate:
	wdr
	sts		TCNT0, ZERO
	ijmp
Phase1: ; B,A
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
	sts		LOW_A, ONE
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_B
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_C)
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


//Actual Loop
Loop:

//Timeout
TimeoutChecker:
	sbrs	TIMEOUT_CNT, 0
	rjmp	TimeoutCheckerEnd

	clr		TIMEOUT_CNT
	clr		COMM_CNT
	cbr		FLAGS, (1 << RUNNING_MODE)
	



	; Startup duty cycle
	ldi		DUTY_H, (START_DUTY >> 8)
	ldi		DUTY_L, (START_DUTY & 255)
	ldi		COMP_DUTY_H, ((TOP-START_DUTY) >> 8)
	ldi		COMP_DUTY_L, ((TOP-START_DUTY) & 255)
	sts		OCR0SBH, COMP_DUTY_H
	sts		OCR0SBL, COMP_DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L





	rjmp	Commutate
TimeoutCheckerEnd:

//Sample comparator
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

//Startup ZC Filter
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

//Running ZC Filter
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

ZCFilterEnd:
	clr		SAMPLE_SUM
	rjmp	Loop

//ZC Event
ZCEvent:
RunningMode:
	sbrs	FLAGS, RUNNING_MODE
	rjmp	StartupMode
	
	sts		TCNT0, ZERO
	asr		TCNT0_TMP ; Divide by 2
	sts		OCR0B, TCNT0_TMP
	sbr		FLAGS, (1 << COMMUTATE_FLAG)
	rjmp	Loop
StartupMode:
	inc		COMM_CNT
	;sbrs	COMM_CNT, 3 ; 8
	;sbrs	COMM_CNT, 4 ; 16
	sbrs	COMM_CNT, 5 ; 32
	;sbrs	COMM_CNT, 6 ; 64
	;sbrs	COMM_CNT, 7 ; 128
	rjmp	Commutate
TransitionMode:
	; Idle duty cycle
	ldi		DUTY_H, (IDLE_DUTY >> 8)
	ldi		DUTY_L, (IDLE_DUTY & 255)
	ldi		COMP_DUTY_H, ((TOP-IDLE_DUTY) >> 8)
	ldi		COMP_DUTY_L, ((TOP-IDLE_DUTY) & 255)
	sts		OCR0SBH, COMP_DUTY_H
	sts		OCR0SBL, COMP_DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L

	sbr		FLAGS, (1 << RUNNING_MODE)
	rjmp	RunningMode

//Interrupt Handlers
//Running Mode Commutation Timer
TIMER0_COMPB_vect:
	sbrs	FLAGS, COMMUTATE_FLAG
	reti
	cbr		FLAGS, (1 << COMMUTATE_FLAG)
	rjmp	Commutate

//Timeout
WDT_vect:
	inc		TIMEOUT_CNT
	reti

//Direction Control
INT1_vect:
	; Check INT1 pin (PB2), manage FLAGS, (1 << DIRECTION)
	sbr		FLAGS, (1 << DIRECTION)
	sbis	_SFR_IO_ADDR(PINB), PB2
	cbr		FLAGS, (1 << DIRECTION)
	reti

//Control via SPI
SPI_STC_vect:
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
	
	sts		OCR0SBH, COMP_DUTY_H
	sts		OCR0SBL, COMP_DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L
SPI_STC_vect_end:
	; Next byte for SPI master (speed)
	;ldi		ISR_RMP, 204
	sts		SPDR, ISR_RMP

	out		_SFR_IO_ADDR(SREG), SREG_TEMP ; Not always necessary
	reti