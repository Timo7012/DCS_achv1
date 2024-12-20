; This example program demonstrates different macros to send data to UART from Arduino Uno.

.include "m328pdef.inc"

.cseg
.org 0x0000
jmp INTI

; Addr of Timer2 Intrerupt Handler
.org 0x0012 ;OVF2addr
jmp TIMER2_OVF_ISR

; Addr of Timer1 Intrerupt Handler
.org 0x001A ;OVF1addr
jmp TIMER1_OVF_ISR

; Start of program
INTI:
	.equ PRESCALER = 1024 ; Prescaler

	.equ TIMER1_INTERVAL = 10 ;Interval of Timer1 in ms, MAX - 4000 ms
	.equ TIMER2_INTERVAL = 10 ;Interval of Timer2 in ms, MAX - 16 ms

; Initial counts of Timers
	.equ TIMER1_START = 65535 - (TIMER1_INTERVAL * 16000000 / PRESCALER / 1000) ; 2^16 = 65536
	.equ TIMER2_START = 255 - (TIMER2_INTERVAL * 16000000 / PRESCALER / 1000)	; 2^8 = 256

; Strings for UART
	TIMER1_STR :  .db "Ping", 0x0D, 0x0A, 0
	TIMER2_STR :  .db "Pong", 0x0D, 0x0A, 0


; Stack Init
	ldi r16, high(RAMEND)
	out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16

	Serial_begin:
	; Initialize UART to 9600 baud
	ldi             R16, LOW(103)
	ldi             R17, HIGH(103)
	sts             UBRR0L, R16
	sts             UBRR0H, R17
	; Enable transmitter and reciever modes
	ldi             R16, (1<<TXEN0)|(1<<RXEN0)
	sts             UCSR0B, R16
	; Set parity to none and 8 data bits, etc.
	ldi             R16, (1<<UCSZ01)|(1<<UCSZ00)
	sts             UCSR0C, R16

    ;Timer1 Init (16-bit)
    ldi r16, high(TIMER1_START)  ; Load init counts of Timer1
    sts TCNT1H, r16              
    ldi r16, low(TIMER1_START)   
    sts TCNT1L, r16              
    ldi r16, (1 << CS12) | (1 << CS10) ; Prescaler = 1024 (TCCR1B: CS12 = 1, CS11 = 0, CS10 = 1)
    sts TCCR1B, r16              
    ldi r16, (1 << TOIE1)        ; Enable interupt when overflow
    sts TIMSK1, r16

    ;Timer2 Init (8-bit)
    ldi r16, TIMER2_START		 ; Load init counts of Timer1
    sts TCNT2, r16				
    ldi r16, (1 << CS22) | (1 << CS21) | (1 << CS20) ; Prescaler = 1024 (TCCR2B: CS22 = 1, CS21 = 1, CS20 = 1)
    sts TCCR2B, r16
    ldi r16, (1 << TOIE2)        ; Enable interupt when overflow
    sts TIMSK2, r16            

    sei                          ; Enable global interrupts

; Main Loop
loop:
    jmp loop


; UART functions
put_char:	
	lds	r16,UCSR0A			; Load UCSR0A into r17
	sbrs r16,UDRE0			; Wait for empty transmit buffer
	rjmp put_char			; Repeat loop
	sts	UDR0,r16			; Transmit character
	ret

puts:	
	lpm	r16,Z+					; Load character from pmem
	cpi	r16, $00				; Check if null
	breq	puts_end			; Branch if null

puts_wait:
	lds		r17,UCSR0A			; Load UCSR0A into r17
	sbrs	r17,UDRE0			; Wait for empty transmit buffer
	rjmp	puts_wait			; Repeat loop

	sts	UDR0,r16				; Transmit character
	rjmp	puts				; Repeat loop

puts_end:
	ret

getc:	
	lds	r17,UCSR0A			; Load UCSR0A into r17
	sbrs	r17,UDRE0		; Wait for empty transmit buffer
	rjmp	getc			; Repeat loop

	sts	UDR0,r16			; Get received character

	ret						; Return from subroutine

gets:	
lds	r17,UCSR0A					; Load UCSR0A into r17
	sbrs	r17,UDRE0			; Wait for empty transmit buffer
	rjmp	put_char			; Repeat loop

	sts	UDR0,r16				; Get received character

	cpi	r16, $0D				; Check if rcv'd char is CR
	breq	gets_end			; Branch if CR rcv'd

	st	X+,r16					; Store character to buffer
	rjmp	gets				; Get another character

gets_end:
	ret
;----------------------


; Interupt Timer1 Handler
TIMER1_OVF_ISR:
    push r16                     
    ldi ZH, high(TIMER1_STR * 2) ; Load "Ping" to Z pointer
    ldi ZL, low(TIMER1_STR * 2)
	rcall	puts				 ; Write in UART	
    ldi r16, high(TIMER1_START)  ; Load init counts of Timer1
    sts TCNT1H, r16              
    ldi r16, low(TIMER1_START)   
    sts TCNT1L, r16              
    pop r16                     
    reti						 ; Return from interupt

; Interupt Timer2 Handler
TIMER2_OVF_ISR:
    push r16                     
    ldi ZH, high(TIMER2_STR * 2) ; Load "Pong" to Z pointer
    ldi ZL, low(TIMER2_STR * 2)
	;rcall	puts				 ; Write in UART	
    ldi r16, TIMER2_START        ; Load init counts of Timer2
    sts TCNT2, r16              
    pop r16                      
    reti						 ; Return from interupt
