.include "tn2313adef.inc" 

; Start macro.inc =========================================

.include "macro.inc"

; #define I2C_DEBUG

.equ PORT_USI = PORTB
.equ DDR_USI  = DDRB
.equ SCL_PIN  = 7
.equ SDA_PIN  = 5

.equ USISR_8bit = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0<<USICNT0)
.equ USISR_1bit = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0xe<<USICNT0)

.equ LIS_ADDRESS = 0x19

; Config constants
; CTRL_REG1
.equ LIS_NORMAL_POWER = 5
.equ LIS_ODR = 3
.equ LIS_Z_EN = 2
.equ LIS_Y_EN = 1
.equ LIS_X_EN = 0

;CTRL_REG4
.equ LIS_BDU = 7
.equ LIS_BLE = 6
.equ LIS_FS = 4
.equ LIS_ST_SIGN = 2
.equ LIS_ST = 1
.equ LIS_SIM = 0

; Registers addresses
.equ LIS_CTRL_REG1 = 0x20
.equ LIS_X_L = 0x28
.equ LIS_Y_L = 0x2a
.equ LIS_Z_L = 0x2c
 
; End macro.inc ===========================================
 
 
; RAM =====================================================
.DSEG

	TCNT: .byte 47
	TX_ptr: .byte 2

	X_MSB: .byte 1
	X_LSB: .byte 1

	Y_MSB: .byte 1
	Y_LSB: .byte 1

	Z_MSB: .byte 1
	Z_LSB: .byte 1


; FLASH =================================================== 
.CSEG

Interrupt_Table:
	.org $000		; (RESET) 
	rjmp Reset

	.org $001		; (INT0) External Interrupt Request 0
	reti

	.org $002		; (INT1) External Interrupt Request 1
	reti

	.org $003		; (TIMER1 CAPT) Timer/Counter1 Capture Event
	reti

	.org $004		; (TIMER1 COMPA) Timer/Counter1 Compare Match A
	reti

	.org $005		; (TIMER1 OVF) Timer/Counter1 Overflow
	reti

	.org $006		; (TIMER0 OVF) Timer/Counter0 Overflow
	rjmp Timer0_OVF

	.org $007		; (USART0, RX) USART0, Rx Complete
	rjmp RX_OK
	; reti

	.org $008		; (USART0, UDRE) USART0 Data Register Empty
	rjmp UDR_Emp
	; reti

	.org $009		; (USART0, TX) USART0, Tx Complete
	rjmp TX_OK
	; reti

	.org $00a		; (ANALOG COMP) Analog Comparator
	reti

	.org $00b		; (PCINT0) Pin Change Interrupt Request 0
	reti

	.org $00c		; (TIMER1 COMPB) Timer/Counter1 Compare Match B
	reti

	.org $00d		; (TIMER0 COMPA) Timer/Counter0 Compare Match A
	reti

	.org $00e		; (TIMER0 COMPB) Timer/Counter0 Compare Match B
	reti

	.org $00f		; (USI START) USI Start Condition
	reti

	.org $010		; (USI OVERFLOW) USI Overflow
	reti

	.org $011		; (EE READY) EEPROM Ready
	reti

	.org $012		; (WDT OVERFLOW) Watchdog Timer Overflow
	reti

	.org $013		; (PCINT1) Pin Change Interrupt Request 1
	reti

	.org $014		; (PCINT2) Pin Change Interrupt Request 2
	reti

	.org INT_VECTORS_SIZE		; End of table of interaptions


; Internal Modules ========================================

.include "uart.inc"
.include "coreinit.inc"

; Interrupts ==============================================

Timer0_OVF:
	pushf
	push r17
	push r18
	push r19

	incm TCNT

	pop r19
	pop r18
	pop r17
	popf

	reti


RX_OK:
	rcall RX_OK_Int
	reti


TX_OK:
	rcall TX_OK_Int
	reti


UDR_Emp:
	rcall UDR_Emp_Int
	reti

; End Interrupts ==========================================
 
 
Reset:
	;Memory, Registers and Stack initialization
	RAM_Flush

; Internal Hardware Init  =================================

	setbm DDRD, 6 ;PB6 is output
	setbm PORTD, 6

	setbm TIMSK, TOIE0
	outi TCCR0, 1 << CS00

	; UART
	rcall UART_Init

	rcall I2C_Init

	sei

; End Internal Hardware Init ==============================



; External Hardware Init  =================================

	; LIS331DLH Init
	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 0
	rcall I2C_Trasmit

	ldi r16, (1 << 7) | LIS_CTRL_REG1
	rcall I2C_Trasmit

	ldi r16, (1 << LIS_NORMAL_POWER) | (0 << LIS_ODR) | (1 << LIS_Z_EN) | (1 << LIS_Y_EN) | (1 << LIS_X_EN) ; CTRL_REG1
	rcall I2C_Trasmit

	ldi r16, 0 			; CTRL_REG2
	rcall I2C_Trasmit

	ldi r16, 0 			; CTRL_REG3
	rcall I2C_Trasmit

	ldi r16, (1 << LIS_BDU) | (1 << LIS_BLE) | (0 << LIS_FS) | (0 << LIS_ST_SIGN) | (0 << LIS_ST) | (0 << LIS_SIM) ; CTRL_REG4
	rcall I2C_Trasmit

	rcall I2C_SendStop	


	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop


	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 0
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp Read_NACK

	ldi r16, (1 << 7) | 0x20
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp Read_NACK

	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 1
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp Read_NACK

	clr r25

Read_Loop:
	ldi r16, 0x0
	rcall I2C_Receive

	rcall Bin2BSD_8
	rcall PrintBSD8

	ldi r19, 13
	rcall Print

	inc r25
	cpi r25, 24
	brne Read_Loop

	ldi r16, 0x1
	rcall I2C_Receive

	rjmp Read_Exit

Read_NACK:
	mprintln "READ NACK"

Read_Exit:
	rcall I2C_SendStop

 
; End External Hardware Init ==============================


 
; Run =====================================================
 
; End Run =================================================



; Run =====================================================

; End Run =================================================



; Main ====================================================

	mprintln "START v.6"

	clrb PORTD, 6, r16

Main:
	lds r16, TCNT
	lds r17, TCNT+1

	cpi r16, 0x42
	brcs NoMatch_Short
	; cpi r17, 0x0f
	cpi r17, 0x04
	brcs NoMatch_Short

	rjmp Match

NoMatch_Short:
	rjmp NoMatch

Match:
	; invbm PORTD, 6
	; rjmp END_LOOP

	rcall I2C_SendStart

	;Transmit
	ldi r16, LIS_ADDRESS << 1 | 0
	rcall I2C_Trasmit
	sbrc r16, 0
	; ; rjmp PC - 3
	rjmp NACK

	ldi r16, (1 << 7) | LIS_X_L
	rcall I2C_Trasmit
	; sbrc r16, 0
	; ; ; rjmp PC - 3
	; rjmp NACK

	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 1
	rcall I2C_Trasmit	
	sbrc r16, 0
	; rjmp PC - 3
	rjmp NACK


	; ================ READ ALL ==============
	ldi r16, 0x0
	rcall I2C_Receive
	sts X_MSB, r16

	ldi r16, 0x0
	rcall I2C_Receive
	sts X_LSB, r16


	ldi r16, 0x0
	rcall I2C_Receive
	sts Y_MSB, r16

	ldi r16, 0x0
	rcall I2C_Receive
	sts Y_LSB, r16


	ldi r16, 0x0
	rcall I2C_Receive
	sts Z_MSB, r16

	ldi r16, 0x1
	rcall I2C_Receive
	sts Z_LSB, r16

	rcall I2C_SendStop

	; ============= PRINT ================
	lds r16, X_MSB
	rcall Bin2BSD_8
	rcall PrintBSD8

	ldi r19, ' '
	rcall Print

	lds r16, X_LSB
	rcall Bin2BSD_8
	rcall PrintBSD8
			
	ldi r19, '\t'
	rcall Print
	ldi r19, '\t'
	rcall Print


	lds r16, Y_MSB
	rcall Bin2BSD_8
	rcall PrintBSD8

	ldi r19, ' '
	rcall Print

	lds r16, Y_LSB
	rcall Bin2BSD_8
	rcall PrintBSD8
			
	ldi r19, '\t'
	rcall Print
	ldi r19, '\t'
	rcall Print


	lds r16, Z_MSB
	rcall Bin2BSD_8
	rcall PrintBSD8

	ldi r19, ' '
	rcall Print

	lds r16, Z_LSB
	rcall Bin2BSD_8
	rcall PrintBSD8

	rjmp I2C_STOP

NACK:
	; mprintln "NACK"
	rcall I2C_SendStop
	rjmp Match

I2C_STOP:
	; rcall I2C_SendStop

	; rcall I2C_SendStart
	; ldi r16, LIS_ADDRESS << 1 | 0
	; rcall I2C_Trasmit
	; ldi r16, 0x0f
	; rcall I2C_Trasmit	
	; rcall I2C_SendStop

	; pop r16

	ldi r19, 13
	rcall Print	

END_LOOP:
	clr r16
	cli

	outu TCNT0, r16
	sts  TCNT, r16
	sts  TCNT+1, r16
	sts	 TCNT+2, r16
	sts	 TCNT+3, r16

	sei

NoMatch:

	rjmp Main
; End Main ================================================



; Procedure ===============================================

; --------------------------------------------
; I2C_Init
;
; IN\OUT: None
; --------------------------------------------
I2C_Init:
#ifdef I2C_DEBUG
	mprintln "INIT"
#endif

	setb PORT_USI, SCL_PIN, r16
	setb PORT_USI, SDA_PIN, r16

	; clrb PORT_USI, SCL_PIN, r16
	; clrb PORT_USI, SDA_PIN, r16

	setb DDR_USI, SCL_PIN, r16
	setb DDR_USI, SDA_PIN, r16

	; clrb DDR_USI, SCL_PIN, r16
	; clrb DDR_USI, SDA_PIN, r16

	outi USIDR, 0xff
	outi USICR, (1<<USIWM1) | (1<<USICS1) | (0<<USICS0) | (1<<USICLK) | (0<<USITC)
	outi USISR, (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC) | (0x0<<USICNT0)
	ret


; --------------------------------------------
; I2C_Delay
;
; IN\OUT: None
; --------------------------------------------
I2C_Delay:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	ret


; --------------------------------------------
; I2C_Wait4SCL
;
; IN\OUT: None
; --------------------------------------------
I2C_Wait4SCL:
	sbis PORT_USI, SCL_PIN
	rjmp I2C_Wait4SCL
	ret


; --------------------------------------------
; I2C_Transeive
;
; IN: r16 - USISR
; OUT: r16 - data
; --------------------------------------------
I2C_Transeive:
	outu USISR, r16

	ldi r17, (1<<USIWM1) | (1<<USICS1) | (0<<USICS0) | (1<<USICLK) | (1<<USITC)

i2c_transeive_loop:
	rcall I2C_Delay
	outu USICR, r17
	rcall I2C_Wait4SCL
	rcall I2C_Delay
	outu USICR, r17

	sbis USISR, USIOIF
	rjmp i2c_transeive_loop

	rcall I2C_Delay
	in r16, USIDR

	ldi r17, 0xff
	out USIDR, r17

	setb DDR_USI, SDA_PIN, r17
	ret


; --------------------------------------------
; I2C_Trasmit
;
; IN:  r16 - data
; OUT: r16 - 0 OK, 1 Error
; --------------------------------------------
I2C_Trasmit:
#ifdef I2C_DEBUG
	push r16
	mprintln "Transmit"
	pop r16
#endif
	clrb PORT_USI, SCL_PIN, r17
	outu USIDR, r16

	ldi r16, USISR_8bit
	rcall I2C_Transeive

	clrb DDR_USI, SDA_PIN, r16

#ifdef I2C_DEBUG
	mprintln "Reading (N)ACK"
#endif
	ldi r16, USISR_1bit
	rcall I2C_Transeive

	sbrs r16, 0
	rjmp  i2c_trasmit_ack

i2c_trasmit_nack:
#ifdef I2C_DEBUG
	push r16
	mprintln "Get NACK"
	pop r16
#endif
	
	rjmp i2c_trasmit_exit

i2c_trasmit_ack:
#ifdef I2C_DEBUG
	push r16
	mprintln "Get ACK"
	pop r16
#endif

i2c_trasmit_exit:	
	andi r16, 0x01
	ret


; --------------------------------------------
; I2C_Receive
;
; IN:  r16 - 0x0 (ACK) or 0x1 (NACK)
; OUT: r16 - data
; --------------------------------------------
I2C_Receive:
	push r16						; Save ACK/NACK input

#ifdef I2C_DEBUG
	push r16
	mprintln "Read"
	pop r16
#endif

	clrb DDR_USI, SDA_PIN, r17

	ldi r16, USISR_8bit
	rcall I2C_Transeive

	pop  r18						; Get ACK/NACK input
	push r16						; Save read data

#ifdef I2C_DEBUG
	push r16
	mprintln "Sending (N)ACK"
	pop r16
#endif

	outu USIDR, r18
	ldi r16, USISR_1bit
	rcall I2C_Transeive

	pop r16
	ret


; --------------------------------------------
; I2C_SendStart
;
; IN\OUT: None
; --------------------------------------------
I2C_SendStart:
#ifdef I2C_DEBUG
	mprintln "START"
#endif

	; clrb DDR_USI, SDA_PIN, r16 ; SDA - high ####
	setb PORT_USI, SDA_PIN, r16; SDA - high

	setb PORT_USI, SCL_PIN, r16 ; SCL - high
	; clrb DDR_USI, SCL_PIN, r16
	rcall I2C_Wait4SCL
	rcall I2C_Delay

	clrb PORT_USI, SDA_PIN, r16 ; SDA - low

	rcall I2C_Delay
	clrb PORT_USI, SCL_PIN, r16 ; SCL - low
	setb PORT_USI, SDA_PIN, r16 ; SDA - high
	ret


; --------------------------------------------
; I2C_SendStop
;
; IN\OUT: None
; --------------------------------------------
I2C_SendStop:
#ifdef I2C_DEBUG
	mprintln "STOP"
#endif

	clrb PORT_USI, SDA_PIN, r16 ; SDA - low
	setb PORT_USI, SCL_PIN, r16 ; SCL - high
	rcall I2C_Wait4SCL
	rcall I2C_Delay
	setb PORT_USI, SDA_PIN, r16 ; SDA - high
	; clrb PORT_USI, SCL_PIN, r16 ; SCL - low
	ret


; --------------------------------------------
; Bin2BSD_8
; Good description of this algorithm
; http://chipmk.ru/index.php?option=com_content&view=article&id=153:2011-07-01-12-21-55
;
; IN:  r16 - binary number
;
; OUT: r17 - BSD low
;	   r18 - BSD high
; --------------------------------------------
Bin2BSD_8:
	clr r17 ; BSD low
	clr r18 ; BSD high

	clr r21 ; Counter

Bin2BSD_8_loop:
	lsl r16
	rol r17
	rol r18

	inc r21
	cpi r21, 8 				; Continue until all 8 bits won't be shifted
	brge Bin2BSD_8_done

	mov r19, r17
	mov r20, r17

	andi r19, 0x0f			 ; Chech low half-byte
	cpi r19, 0x5			 ; If correction has to be done
	brlo Bin2BSD_8_4_or_less ; If low half-byte equal to 4 or less, there's no need for correction

	ldi r19, 0x3 			 ; Othervise add 0x3 as correction
	add r17, r19

Bin2BSD_8_4_or_less:
	andi r20, 0xf0			 ; Check high half-byte
	cpi r20, 0x41			 ; If correction has to be done
	brlo Bin2BSD_8_loop		 ; If high half-byte equal to 4 or less, there's no need for correction

	ldi r20, 0x30			 ; Othervise add 0x3 as correction
	add r17, r20

	rjmp Bin2BSD_8_loop

Bin2BSD_8_done:
	ret


; --------------------------------------------
; Bin2BSD_16
; Good description of this algorithm
; http://chipmk.ru/index.php?option=com_content&view=article&id=153:2011-07-01-12-21-55
;
; IN:  r16 - binary number low
; 	   r17 - binary number high
;
; OUT: r18 - BSD low
;	   r19 - BSD mid
;	   r20 - BSD high
; --------------------------------------------
Bin2BSD_16:
	clr r18 ; BSD low
	clr r19 ; BSD mid
	clr r20 ; BSD high

	clr r25 ; Counter

Bin2BSD_16_loop:
	lsl r16
	rol r17

	rol r18
	rol r19
	rol r20

	inc r25
	cpi r25, 16 				; Continue until all 16 bits won't be shifted
	brge Bin2BSD_16_done

	mov r21, r18
	mov r22, r18
	mov r23, r19
	mov r24, r19

	andi r21, 0x0f				; Chech low half-byte of BSD low
	cpi r21, 0x5				; If correction has to be done
	brlo Bin2BSD_16_4_or_less 	; If low half-byte equal to 4 or less, there's no need for correction

	ldi r21, 0x3 				; Othervise add 0x3 as correction
	add r18, r21

Bin2BSD_16_4_or_less:
	andi r22, 0xf0				; Same for high half-byte of BSD low
	cpi r22, 0x41
	brlo Bin2BSD_16_40_or_less

	ldi r22, 0x30				; Correction
	add r18, r22

Bin2BSD_16_40_or_less:
	andi r23, 0x0f				; Same for low half-byte of BSD high
	cpi r23, 0x5
	brlo Bin2BSD_16_400_or_less

	ldi r23, 0x3 				; Correction (actual correction for BSD word is 0x300)
	add r19, r23

Bin2BSD_16_400_or_less:
	andi r24, 0xf0 				; Same for high half-byte of BSD high
	cpi r24, 0x41
	brlo Bin2BSD_16_loop

	ldi r24, 0x30 				; Correction (actual correction for BSD word is 0x3000)
	add r19, r24

	rjmp Bin2BSD_16_loop

Bin2BSD_16_done:
	ret

; End Procedure ===========================================


; EEPROM =====================================================
	.ESEG


