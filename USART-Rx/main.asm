;***********************************************************
;*
;*	Daniel_Green_Cody_McKenzie_Lab8_Rx_sourcecode.asm
;*
;*	This program initializes and executes USART transmit
;*	functionality. It is meant to communicate with the
;*	Tx program.   
;*
;*	Portions of this code was copied from the ECE375 Textbook 
;*  Lab1 starter code, and the Atmega128 Datasheet
;*
;***********************************************************
;*
;*	 Author: Daniel Green, Cody McKenzie
;*	   Date: 12/3/2019
;*
;***********************************************************

.include "m128def.inc"          ; Include definition file

;***********************************************************
;*  Internal Register Definitions and Constants
;***********************************************************
.def    mpr = r16               ; Multi-Purpose Register
.def	recv_data = r19			; Reg for holding recieved USART data
.def	cmd = r20				; Reg for decoded command
.def	frzCnt = r21			; number of recived freeze signals
.def	cmdReadyFlag = r22		; flag to determine if the bot is ready to recieve cmd
.def	waitReg = r23			; holds waitcnt value	 

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit
.equ    BotID = $2B				; ID must match Tx
.equ	freezeSig = 0b01010101		; freeze signal

; Use these action codes between the remote and robot
; MSB = 1 thus:
; control signals are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd =  ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forward Action Code
.equ	MovBck =  ($80|$00)								;0b10000000 Move Backward Action Code
.equ	TurnR =   ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Action Code
.equ	TurnL =   ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Action Code
.equ	Halt =    ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Action Code
.equ	Frz =     0b11111000

;***********************************************************
;*  Start of Code Segment
;***********************************************************
.cseg                           ; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org    $0000                   
        rjmp    INIT

.org    $0002
        rjmp   HitRight ; ISR for BumpBot behavior
        
.org    $0004
        rjmp   HitLeft ; ISR for BumpBot behavior
        
.org    $003C
        rjmp   USART_Receive ; ISR for handling USART Recieve interrupt
        

.org    $0046                   ; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:
	; Initialize stack
	ldi		mpr, high(RAMEND)
	out		SPH, mpr
	ldi		mpr, low(RAMEND)
	out		SPL, mpr

	clr		frzCnt
	clr		cmd
	clr		recv_data
	clr		cmdReadyFlag
	ldi		waitreg, 100 ; load default wait of 1 sec
	
 ;* Initialize I/O Ports
	; Initialize Port B for output
	ldi		mpr, $FF		; Set Port B Data Direction Register
	out		DDRB, mpr		; for output
	ldi		mpr, $00		; Initialize Port B Data Register
	out		PORTB, mpr		; so all Port B outputs are low		

	; Initialize Port D for input
	 ldi mpr, (0<<WskrR)|(0<<WskrL)
     out DDRD, mpr

    ; Set pullup resistors
     ldi mpr, (0<<WskrR)|(0<<WskrL)
     out PORTD, mpr

 ;* Initialize Timers
	; Init TCNT0
	LDI mpr, 0b00000111 ; Activate Normal mode, OC0 disconnected,
	OUT TCCR0, mpr ; and set prescaler to 1024

 ;* Initialize USART
    ; Initialize USART1
        ldi     mpr, (1<<U2X1) ; Set double data rate
        sts     UCSR1A, mpr

        ; Set baud rate at 2400bps (taking into account double data rate)
        ldi     mpr, high(832) ; Load high byte of 0x0340
        sts     UBRR1H, mpr ; UBRR1H in extended I/O space
        ldi     mpr, low(832) ; Load low byte of 0x0340
        sts     UBRR1L, mpr

        ; Set frame format: 8 data, 2 stop bits
        ldi     mpr, (0<<UMSEL1 | 1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10)
        sts     UCSR1C, mpr ; UCSR1C in extended I/O space

        ; Enable USART receiver, transmitter and USART receive interrupts
        ldi     mpr, (1<<RXEN1 | 1<<RXCIE1 | 1<<TXEN1)
        sts     UCSR1B, mpr

  ;* Initialize external interrupts
        ; Set Interrupt Sense Control to falling edge trigger
        ldi mpr, 0b10101010
        sts EICRA, mpr ; Use sts, EICRA in extended I/O space
        ; Set the External Interrupt Mask
        ldi mpr, (1<<INT1)|(1<<INT0)
        out EIMSK, mpr

  ;* Begin BumpBot behaviour, saving cmd
		ldi cmd, MovFwd
		lsl	cmd
		out PORTB, cmd
        
		sei ; Turn on global interrupts

;-----------------------------------------------------------
; Main Program
; There are less possible inputs in this program, so using
; interrupts is easier to manage.
; Portions of this code are taken from the ECE375 Textbook
; and the week 1 bumpbot example code.  
;-----------------------------------------------------------
MAIN:
        rjmp    MAIN

;***********************************************************
;*  Functions and Subroutines
;***********************************************************

;***********************************************************
;*  Sub: USART_Recieve
;*  Des: This ISR is executed when the "USART recieve interrupt"
;*  is triggered. It first loads the data from 
;*  USART Data Register 1 into mpr. Then is checks
;*  if the recieved data is the freeze signal, if so the 
;*  appropriate freeze routine is called. Then is checks
;*  if the recieved data is an ID, if so then the cmdReadyFlag
;*  register is set to indicate that the bot is ready to recieve
;*  a command. If the recieved data is not an ID then it must
;*  be a command and it is executed if the cmdReadyFlag us set.
;***********************************************************
USART_Receive:
		; save program state
        push    mpr
		in		mpr, SREG
		push	mpr
		; Read data from Receive Data Buffer in Extended I/O space
        lds     recv_data, UDR1
		mov		mpr, recv_data

freezeSigCheck:
		; determine if the recieved data is the freeze signal
		cpi		mpr, freezeSig
		brne	idCheck
		inc		frzCnt
		cpi		frzCnt, $3
		; if 3rd time recieving freeze signal go into PermaFreeze
		breq	permaFreeze
		; otherwise freeze for 5 seconds
		rjmp	tempFreeze
				
idCheck:		 
		; Determine if the recieved data is a BotID
        cpi		mpr, 0b10000000
        brne    cmdCheck ; If the zero flag is NOT set, then the recieved data is a Command    
						; otherwise its a BotID
		mov		mpr, recv_data
        cpi     mpr, BotID
        brne    endRecv ; The BotID is not recognized
		ldi		cmdReadyFlag, $1 ; bot is ready to recieve a command
		rjmp	endRecv

cmdCheck:
		; make sure the last recieved data was a valid BotID 
		cpi		cmdReadyFlag, $1
		; if not ready for cmd, go to end
		brne	endRecv 
		mov		mpr, recv_data
        lsl     mpr ; remove the MSB to format for display
        mov     cmd, mpr    
        out     PORTB, cmd  ; Send cmd to PORTB
		clr		cmdReadyFlag
		; check if the recieved cmd is freeze command 
		mov		mpr, recv_data
		cpi		mpr, Frz
		brne	endRecv
		; if cmd is freeze, then send the freeze signal 
		rcall	USART_Transmit
		rjmp	endRecv

; This routine is entered but never exited.
permaFreeze:
		ldi		mpr, Halt ;display halt command on LEDs
		lsl		mpr
		out		PORTB, mpr
		cli ; turn off all interrupts
		dead:
		rjmp dead ; enter an infinite loop

; This routine is entered a total of 3 times
; and execution is paused for 5 seconds. 
tempFreeze:
		lsl		mpr
		out		PORTB, mpr ; display the freeze command on the LEDs 
		cli     ; turn off all interrupts
		ldi		waitreg, 250
		rcall	Wait ; wait for 5 seconds
		rcall	Wait
		ldi		waitreg, 100
		sei     ; turn interrupts back on
		rjmp	endRecv ; return to end of normal execution

; This routine is called at the end of every USART_Recieve execution 		
endRecv:
		; restore program state and do nothing
        pop		mpr
		out		SREG, mpr
		pop		mpr
		reti


;----------------------------------------------------------------
; Sub:  HitRight
; Desc: Handles functionality of the TekBot when the right whisker
;       is triggered.
;		Portions of this code taken from ECE375 Lab 1 starter code
;----------------------------------------------------------------
HitRight:
        push	mpr			; Save mpr register
		in		mpr, SREG	; Save program state
		push	mpr			;

		ldi		mpr, $00
		out		EIMSK, mpr

        ; Move Backwards for a second
        ldi     mpr, $00	; Load Move Backwards command
        out     PORTB, mpr  ; Send command to port
        rcall   Wait            ; Call wait function

        ; Turn left for a second
        ldi     mpr, TurnL  ; Load Turn Left Command
		lsl		mpr
        out     PORTB, mpr  ; Send command to port
        rcall   Wait            ; Call wait function

        ; Return to previous behaviour
        out     PORTB, cmd

		; Clear any queued whisker interrupt flags
		in		mpr, EIFR			
		sbr		mpr, (1<<WskrR)|(1<<WskrL)	
		out		EIFR, mpr	

		ldi		mpr, 0b00000111
		out		EIMSK, mpr

        pop     mpr     ; Restore program state
        out     SREG, mpr   ;
        pop     mpr     ; Restore mpr
        reti             ; Return from subroutine

;----------------------------------------------------------------
; Sub:  HitLeft
; Desc: Handles functionality of the TekBot when the left whisker
;       is triggered.
;----------------------------------------------------------------
HitLeft:
        push    mpr         ; Save mpr register
        in      mpr, SREG   ; Save program state
        push    mpr

		ldi		mpr, $FF
		
		ldi		mpr, $00
		out		EIMSK, mpr         ;

        ; Move Backwards for a second
        ldi     mpr, $00 ; Load Move Backwards command
        out     PORTB, mpr  ; Send command to port
        rcall   Wait        ; Call wait routine - 1 sec

        ; Turn right for a second
        ldi     mpr, TurnR  ; Load Turn right Command
		lsl		mpr
        out     PORTB, mpr  ; Send command to port
        rcall   Wait            ; Call wait function

        ; Return to previous behaviour
        out     PORTB, cmd

		; Clear any queued whisker interrupt flags
		in		mpr, EIFR			
		sbr		mpr, (1<<WskrR)|(1<<WskrL)	
		out		EIFR, mpr	

		ldi		mpr, 0b00000111
		out		EIMSK, mpr

        pop     mpr     ; Restore program state
        out     SREG, mpr   ;
        pop     mpr     ; Restore mpr
        reti             ; Return from subroutine

USART_Transmit:
		push	cmd
		push	mpr
		; don't allow this robot to be frozen by disabling usart reciever
		ldi		mpr, (0<<RXCIE1)|(1<<TXCIE1)|(0<<RXEN1)|(1<<TXEN1)
		sts		UCSR1B, mpr

		lds		mpr, UCSR1A 
		sbrs	mpr, UDRE1 ; Loop until the data register is empty, checking the UDRE bit 
		rjmp	USART_Transmit
		ldi		cmd, freezeSig
		sts		UDR1, cmd ; Move data to Transmit Data Buffer
		rcall	Wait
		; re-enable usart reciever
		ldi     mpr, (1<<RXEN1 | 1<<RXCIE1 | 1<<TXEN1)
        sts     UCSR1B, mpr
		pop		mpr
		pop		cmd
		ret

; Use timer/counter0 with 10ms delay 100 times == 1 sec 
WAIT:
	push	r16
	push	r17
	mov		R17, waitreg ; Load count = 100 
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
;*  Stored Program Data
;***********************************************************



;***********************************************************
;*  Additional Program Includes
;***********************************************************
