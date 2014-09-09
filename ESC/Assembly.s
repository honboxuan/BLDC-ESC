
/*
 * Assembly.s
 *
 * Created: 24/02/2014 23:22:21
 *  Author: Hon Bo Xuan
 */ 

#include <avr/io.h>
#include "ESC.h"

//Free after ASM takes over
#define	ZERO		r1
#define	ONE			r2
#define	COMM_CNT	r3
#define	TCNT0_TMP	r4
#define	TIMEOUT_CNT	r5

//Free-to-use registers: r18 to r27 (.def does not work)
//Use if instructions require r16-r31
#define	RMP					r18
#define	FLAGS				r19
#define	COMPARATOR_MASK		r20
#define	SAMPLE_SUM			r21
#define	SAMPLE_CNT			r22
#define	DUTY_H				r23
#define	DUTY_L				r24

.global Initialise
//.global AllOff
.global Loop
//.global PSC1_EC_vect
.global TIMER0_COMPB_vect
;.global TIMER0_OVF_vect
.global	WDT_vect

Initialise:
	ldi		RMP, 1
	mov		ONE, RMP
	clr		FLAGS
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)
	clr		ZH
	ldi		ZL, pm_lo8(Phase1)
	clr		DUTY_H
	clr		DUTY_L
	sei
AllOff:
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO
	sts		LOW_B, ZERO
	sts		LOW_C, ZERO
	ret

Commutate:
	wdr
	sts		TCNT0, ZERO
	ijmp
Phase1: ; B,A
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO ; Unneccessary
	sts		LOW_B, ZERO ; Unneccessary
	sts		LOW_C, ZERO
	sts		LOW_A, ONE
	
	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_B ; Unneccessary
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_C)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase2) ; Unneccessary
	ldi		ZL, pm_lo8(Phase2)
#else
	ldi		ZH, pm_hi8(Phase6) ; Unneccessary
	ldi		ZL, pm_lo8(Phase6)
#endif

	sbrc	FLAGS, RUNNING_MODE
	reti
	
	rjmp	Loop
Phase2: ; C,A
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_B, ZERO ; Unneccessary
	sts		LOW_C, ZERO ; Unneccessary
	sts		LOW_A, ONE ; Unneccessary

	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_C
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_B)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase3) ; Unneccessary
	ldi		ZL, pm_lo8(Phase3)
#else
	ldi		ZH, pm_hi8(Phase1) ; Unneccessary
	ldi		ZL, pm_lo8(Phase1)
#endif
	
	sbrc	FLAGS, RUNNING_MODE
	reti
	
	rjmp	Loop
Phase3: ; C,B
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO ; Unneccessary
	sts		LOW_A, ZERO
	sts		LOW_C, ZERO ; Unneccessary
	sts		LOW_B, ONE

	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_C ; Unneccessary
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase4) ; Unneccessary
	ldi		ZL, pm_lo8(Phase4)
#else
	ldi		ZH, pm_hi8(Phase2) ; Unneccessary
	ldi		ZL, pm_lo8(Phase2)
#endif
	
	sbrc	FLAGS, RUNNING_MODE
	reti

	rjmp	Loop
Phase4: ; A,B
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO ; Unneccessary
	sts		LOW_C, ZERO ; Unneccessary
	sts		LOW_B, ONE ; Unneccessary

	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_A
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_C)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase5) ; Unneccessary
	ldi		ZL, pm_lo8(Phase5)
#else
	ldi		ZH, pm_hi8(Phase3) ; Unneccessary
	ldi		ZL, pm_lo8(Phase3)
#endif
	
	sbrc	FLAGS, RUNNING_MODE
	reti
	
	rjmp	Loop
Phase5: ; A,C
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO ; Unneccessary
	sts		LOW_A, ZERO ; Unneccessary
	sts		LOW_B, ZERO
	sts		LOW_C, ONE

	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_A ; Unneccessary
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_B)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase6) ; Unneccessary
	ldi		ZL, pm_lo8(Phase6)
#else
	ldi		ZH, pm_hi8(Phase4) ; Unneccessary
	ldi		ZL, pm_lo8(Phase4)
#endif
	
	sbrc	FLAGS, RUNNING_MODE
	reti
	
	rjmp	Loop
Phase6: ; B,C
	out		_SFR_IO_ADDR(HIGH_PORT), ZERO
	sts		LOW_A, ZERO ; Unneccessary
	sts		LOW_B, ZERO ; Unneccessary
	sts		LOW_C, ONE ; Unneccessary

	sbi		_SFR_IO_ADDR(HIGH_PORT), HIGH_B
	ldi		COMPARATOR_MASK, (1 << COMPARATOR_A)

#if DIRECTION == 0
	ldi		ZH, pm_hi8(Phase1) ; Unneccessary
	ldi		ZL, pm_lo8(Phase1)
#else
	ldi		ZH, pm_hi8(Phase5) ; Unneccessary
	ldi		ZL, pm_lo8(Phase5)
#endif

	sbrc	FLAGS, RUNNING_MODE
	reti

	rjmp	Loop

Loop:
/*
//See frequency
	sbi		_SFR_IO_ADDR(PORTB), PB7
	nop
	cbi		_SFR_IO_ADDR(PORTB), PB7
*/
/*
//See comparator output
	lds		RMP, ACSR
	and		RMP, COMPARATOR_MASK
	breq	ComparatorASampleLow
	sbi		_SFR_IO_ADDR(PORTB), PB7
	rjmp	ComparatorSampler
	;rjmp	Loop
ComparatorASampleLow:
	cbi		_SFR_IO_ADDR(PORTB), PB7
	;rjmp	Loop
*/
/*
//See PSC output activity flag
	lds		RMP, PIFR0
	sbrs	RMP, POAC0A
	rjmp	ActivityFlagLow
	sbi		_SFR_IO_ADDR(PORTB), PB7

	ldi		RMP, (1 << POAC0A)
	sts		PIFR0, RMP

	rjmp	ComparatorSampler
	;rjmp	Loop
ActivityFlagLow:
	cbi		_SFR_IO_ADDR(PORTB), PB7
	;rjmp	Loop
*/
/*
//See PSC0 pin status
	sbic	_SFR_IO_ADDR(PIND), PIND0
	rjmp	FETOn
	cbi		_SFR_IO_ADDR(PORTB), PB7
	rjmp	ComparatorSampler
FETOn:
	sbi		_SFR_IO_ADDR(PORTB), PB7
*/




TimeoutChecker:
	sbrs	TIMEOUT_CNT, 0
	rjmp	TimeoutCheckerEnd

	clr		TIMEOUT_CNT
	clr		COMM_CNT
	cbr		FLAGS, (1 << RUNNING_MODE)
	
	; Startup duty cycle
	ldi		DUTY_H, (START_DUTY >> 8)
	ldi		DUTY_L, (START_DUTY & 255)
	sts		OCR0RAH, DUTY_H
	sts		OCR0RAL, DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L

	rjmp	Commutate
TimeoutCheckerEnd:





/*
//Original sampler
ComparatorSampler:
	lds		RMP, ACSR
	and		RMP, COMPARATOR_MASK
	breq	ComparatorSampleLow
ComparatorSampleHigh:
	inc		SAMPLE_SUM
ComparatorSampleLow:
	inc		SAMPLE_CNT
	;sbrs	SAMPLE_CNT, 5 ; 32
	sbrs	SAMPLE_CNT, 6 ; 64
	;sbrs	SAMPLE_CNT, 7 ; 128
	rjmp	Loop
*/









//Check PSC pin status before sampling
//One of them must be high
/*
	sbic	_SFR_IO_ADDR(PIND), PIND0
	rjmp	ComparatorSampler
	sbic	_SFR_IO_ADDR(PINC), PINC0
	rjmp	ComparatorSampler
	sbic	_SFR_IO_ADDR(PINB), PINB0
	rjmp	ComparatorSampler
	rjmp	Loop
*/



//Modified sampler and ZCFilter
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




;Full power startup has no PWM noise
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











/*
//Original working ZCfilter
ZCFilter:
	clr		SAMPLE_CNT
	sbrs	FLAGS, COMPARATOR_STATE
	rjmp	ComparatorStateLow
ComparatorStateHigh:
	cpi		SAMPLE_SUM, 4

	brsh	NoZC
	lds		TCNT0_TMP, TCNT0
	cbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
ComparatorStateLow:
	cpi		SAMPLE_SUM, 30

	brlo	NoZC 
	lds		TCNT0_TMP, TCNT0
	sbr		FLAGS, (1 << COMPARATOR_STATE)
	clr		SAMPLE_SUM
	rjmp	ZCEvent
NoZC:
	clr		SAMPLE_SUM
	rjmp	Loop
*/








ZCEvent:
RunningMode:
	sbrs	FLAGS, RUNNING_MODE
	rjmp	StartupMode

	sbi		_SFR_IO_ADDR(PORTB), PB7
	sts		TCNT0, ZERO

	;asr		TCNT0_TMP
	;asr		TCNT0_TMP
	;asr		TCNT0_TMP
	;asr		TCNT0_TMP
	asr		TCNT0_TMP
	

	;sts		TCNT0, ZERO
	sts		OCR0B, TCNT0_TMP
	sbr		FLAGS, (1 << COMMUTATE_FLAG)

	;ldi		RMP, (1 << OCIE0B)
	;sts		TIMSK0, RMP
	
	cbi		_SFR_IO_ADDR(PORTB), PB7
	rjmp	Loop
StartupMode:
	;sbrc	FLAGS, RUNNING_MODE
	;rjmp	RunningMode
	inc		COMM_CNT
	;sbrs	COMM_CNT, 3 ; 8
	;sbrs	COMM_CNT, 4 ; 16
	sbrs	COMM_CNT, 5 ; 32
	;sbrs	COMM_CNT, 6 ; 64
	;sbrs	COMM_CNT, 7 ; 128
	rjmp	Commutate
	
	;ldi		DUTY_H, 1
	;ldi		DUTY_L, 255
	ldi		DUTY_H, (IDLE_DUTY >> 8)
	ldi		DUTY_L, (IDLE_DUTY & 255)

	sts		OCR0RAH, DUTY_H
	sts		OCR0RAL, DUTY_L
	sts		OCR1RAH, DUTY_H
	sts		OCR1RAL, DUTY_L
	sts		OCR2RAH, DUTY_H
	sts		OCR2RAL, DUTY_L

	sbr		FLAGS, (1 << RUNNING_MODE)
	rjmp	RunningMode





/*
PSC1_EC_vect:
	sbi		_SFR_IO_ADDR(PORTB), PB7
;	nop
	cbi		_SFR_IO_ADDR(PORTB), PB7
	reti
*/




TIMER0_COMPB_vect:
	;sbi		_SFR_IO_ADDR(PORTB), PB7
	;nop
	;nop
	;cbi		_SFR_IO_ADDR(PORTB), PB7
	;reti

	sbrs	FLAGS, COMMUTATE_FLAG
	reti
	cbr		FLAGS, (1 << COMMUTATE_FLAG)
	;sts		TIMSK0, ZERO
	rjmp	Commutate

/*
TIMER0_OVF_vect:
	inc		TIMEOUT_CNT
	reti
*/

WDT_vect:
	inc		TIMEOUT_CNT
	reti