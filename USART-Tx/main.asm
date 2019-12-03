;***********************************************************
;*
;*	Daniel_Green_Cody_McKenzie_Lab8_Tx_sourcecode.asm
;*
;*	This program initializes and executes USART transmit
;*	functionality. It is meant to communicate with the
;*	Rx program.   
;*
;*	Portions of this code was copied from the ECE375 Textbook 
;*	and Lab1 starter code.
;*
;***********************************************************
;*
;*	 Author: Daniel Green, Cody Mckenzie
;*	   Date: 12/3/2019
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	cmd = r17
.def	tmp_cmd = r18

.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit
; Use these action codes between the remote and robot
; MSB = 1 thus:
; control signals are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forward Action Code
.equ	MovBck =  ($80|$00)								;0b10000000 Move Backward Action Code
.equ	TurnR =   ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Action Code
.equ	TurnL =   ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Action Code
.equ	Halt =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Action Code
.equ	Frz =	  $F8									;0b11111000 Freeze Action Code

; Id of TA Rx for testing Tx 
.equ	BotID =	$2B; 0b00101010

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt
		
.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
	; Initialize stack
	ldi		mpr, high(RAMEND)
	out		SPH, mpr
	ldi		mpr, low(RAMEND)
	out		SPL, mpr
	
 ;* Initialize I/O Ports
	; Initialize Port B for output
	ldi		mpr, $FF		; Set Port B Data Direction Register
	out		DDRB, mpr		; for output
	ldi		mpr, $00		; Initialize Port B Data Register
	out		PORTB, mpr		; so all Port B outputs are low		

	; Initialize Port D for input
	ldi		mpr, 0b00000000	; Set Port D Data Direction Register
	out		DDRD, mpr		; for input
	ldi		mpr, 0b11110011	; Initialize Port D Data Register
	out		PORTD, mpr		; so all Port D inputs are Tri-State

 ;* Initialize Timers
	; Init TCNT0
	LDI mpr, 0b00000111 ; Activate Normal mode, OC0 disconnected,
	OUT TCCR0, mpr ; and set prescaler to 1024

 ;* Initialize USART
	; Initialize USART1
	ldi mpr, (1<<U2X1) ; Set double data rate
	sts UCSR1A, mpr ;

	; Set baudrate at 2400
	ldi mpr, high(832) ; Load high byte of 0x0340 - double data rate
	sts UBRR1H, mpr ; UBRR0H in extended I/O space
	ldi mpr, low(832) ; Load low byte of 0x0340
	sts UBRR1L, mpr ;

	; Set frame format: 8 data, 2 stop bits, asynchronous
	ldi mpr, (0<<UMSEL1 | 1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10)
	sts UCSR1C, mpr ; UCSR1C in extended I/O space

	; Enable transmitter
	ldi mpr, (1<<TXEN1)
	sts UCSR1B, mpr ;
	
	sei ; Enable global interrupt

;***********************************************************
;*	Main Program
;  Instead of reading interrupts, the main loop simply polls
;  PIND for inputs that match known commands.
;  This code is adapted from the whisker input code from week1.
;***********************************************************
MAIN:
		in		tmp_cmd, PIND	; Get input from Port D 
		andi	tmp_cmd, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD6|1<<PD7)
		rcall	NEXT_F
		rjmp	MAIN
NEXT_F:	
		mov		mpr, tmp_cmd	; copy input to mpr for pocessing
		cpi		mpr, (1<<PD1|1<<PD4|1<<PD5|1<<PD6|1<<PD7)		; Check for PD0 input
		brne	NEXT_B			; Continue with next check for backward command
		rcall	Forward			; Call the subroutine Forward
		ret					
NEXT_B:	
		mov		mpr, tmp_cmd	
		cpi		mpr, (1<<PD0|1<<PD4|1<<PD5|1<<PD6|1<<PD7) ; check for PD1 input
		brne	NEXT_R
		rcall	Backward		; call subroutine Backward
		ret			
	;PD2 and PD3 are reserved for USART
NEXT_R:
		mov		mpr, tmp_cmd
		cpi		mpr, (1<<PD0|1<<PD1|1<<PD5|1<<PD6|1<<PD7) ; check for PD4 input
		brne	NEXT_L			 
		rcall	Right			; call subroutine Right
		ret
NEXT_L:
		mov		mpr, tmp_cmd
		cpi		mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD6|1<<PD7) ; check for PD5 input
		brne	NEXT_H
		rcall	Left			; call subroutine Left
		ret
NEXT_H:
		mov		mpr, tmp_cmd
		cpi		mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD7) ; check for PD6 input
		brne	NEXT_FRZ
		rcall	Stop			; call subroutine Stop
		ret
NEXT_FRZ:
		mov		mpr, tmp_cmd
		cpi		mpr, (1<<PD0|1<<PD1|1<<PD4|1<<PD5|1<<PD6) ; check for PD7 input
		sbrs	mpr, 7
		rcall	Freeze
		ret
;***********************************************************
;*	Functions and Subroutines
;***********************************************************

Forward:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Forward cmd on Tx
		ldi		mpr, MovFwd			; Load Forward command
		out		PORTB, mpr			; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Forwardcmd to cmd reg and transmit
		ldi		cmd, MovFwd
		rcall	USART_Transmit

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

Backward:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Backward cmd on Tx
		ldi		mpr, MovBck		; Load Backward command
		out		PORTB, mpr		; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Backwardcmd to cmd reg and transmit
		ldi		cmd, MovBck
		rcall	USART_Transmit
		  
		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

Right:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Turn Right cmd on Tx
		ldi		mpr, (TurnR)		; Load Turn Right command
		out		PORTB, mpr		; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Turn Right to cmd reg and transmit
		ldi		cmd, TurnR
		rcall	USART_Transmit

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

Left:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Turn Left cmd on Tx
		ldi		mpr, TurnL		; Load Turn Left command
		out		PORTB, mpr		; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Turn Left to cmd reg and transmit
		ldi		cmd, TurnL
		rcall	USART_Transmit

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

Stop:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Halt cmd on Tx
		ldi		mpr, Halt		; Load Halt command
		out		PORTB, mpr		; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Haltcmd to cmd reg and transmit
		ldi		cmd, Halt
		rcall	USART_Transmit

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

Freeze:
		push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Display Freeze cmd on Tx LED
		ldi		mpr, Frz		; Load Frz command
		out		PORTB, mpr		; Send command to port

		; Copy the botID to cmd reg and transmit
		ldi		cmd, BotID
		rcall	USART_Transmit

		; Copy Haltcmd to cmd reg and transmit
		ldi		cmd, Frz
		rcall	USART_Transmit

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		mpr		; Restore mpr
		ret				; Return from subroutine
		
USART_Transmit:
		lds		mpr, UCSR1A 
		sbrs	mpr, UDRE1 ; Loop until the data register is empty, checking the UDRE bit 
		rjmp	USART_Transmit
		sts		UDR1, cmd ; Move data to Transmit Data Buffer
		ret
		
; Use timer/counter0 with 10ms delay 50 times 
WAIT:
	push	r16
	push	r17
	LDI		R17, 30 ; Load count = 30 
WAIT_10msec:
	LDI		R16, 100 ; Value for delay = 100
	OUT		TCNT0, R16 ; (Re)load a value for delay
LOOP:
	IN		R18,TIFR ; Read in TOV0
	ANDI	R18, 0b00000001 ; Check if its set
	BREQ	LOOP ; Loop if TOV0 not set
	LDI		R18, 0b00000001 ; Reset TOV0
	OUT		TIFR, R18 ; Note - write 1 to reset
	DEC		R17 ; Decrement count
	BRNE	WAIT_10msec ; Loop if count not equal to 0
	pop		r17
	pop		r16
	RET	
;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************