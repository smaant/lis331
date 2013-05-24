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

.equ Threshold = 15
 
; End macro.inc ===========================================
 
 
; RAM =====================================================
.DSEG

	MC_CNT: .byte 4 	; Main Cycle counter
	REST_CNT: .byte 4 	; Counter for resting time

	TX_ptr: .byte 2

	X_MSB: .byte 1
	X_LSB: .byte 1

	Y_MSB: .byte 1
	Y_LSB: .byte 1

	Z_MSB: .byte 1
	Z_LSB: .byte 1

	X_MSB_prev: .byte 1
	Y_MSB_prev: .byte 1
	Z_MSB_prev: .byte 1
	Prev_diff:  .byte 1


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
.include "i2c.inc"

; Interrupts ==============================================

Timer0_OVF:
	pushf
	push r17
	push r18
	push r19

	incm MC_CNT
	incm REST_CNT

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

	ldi r16, 137
	rcall abs_print

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
	rcall LIS_Init

	; rcall I2C_Delay

	; rcall LIS_PrintMemory


 
; End External Hardware Init ==============================


 
; Run =====================================================
 
; End Run =================================================



; Run =====================================================

; End Run =================================================



; Main ====================================================

	mprintln "START v.6"

	clrb PORTD, 6, r16

Main:
	lds r16, MC_CNT
	lds r17, MC_CNT+1

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

	rcall LIS_Update
	rcall LIS_TestPrint

	rjmp END_LOOP

	clr r18 ; sign

	lds r16, X_MSB
	lds r17, X_MSB_prev
	sts X_MSB_prev, r16
	sub r16, r17

	brlt Negative
	rjmp END_LOOP

Negative:
	brvs Overflow

	ser r18 ; sign

	; abs
	subi r16, 1
	ser  r17
	eor  r16, r17

	rjmp END_LOOP

Overflow:
	ser r17
	sub r17, r16
	inc r17
	mov r16, r17

	ser r18 ; sign

END_LOOP:
	; push r16

; 	sbrs r18, 0 	; Negative?
; 	rjmp Positive

; 	ldi r19, '-'
; 	rcall Print

; Positive:
; 	rcall Bin2BSD_8
; 	rcall PrintBSD8

; 	; ldi r19, ' '
; 	; rcall Print

; 	; lds r16, Prev_diff	

; 	; rcall Bin2BSD_8
; 	; rcall PrintBSD8

; 	ldi r19, 13
; 	rcall Print


	clr r16

	cli

	outu TCNT0, r16
	sts  MC_CNT, r16
	sts  MC_CNT + 1, r16
	sts	 MC_CNT + 2, r16
	sts	 MC_CNT + 3, r16

	sei

NoMatch:

	rjmp Main
; End Main ================================================



; Procedure ===============================================


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



; --------------------------------------------
; LIS_Update
; Read X, Y and Z acceleration values and store them in a memory
;
; IN:	None

; OUT:	X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
;		These variable have to be defined the DSEG
; --------------------------------------------
LIS_Update:
	clr r16
	push r16	; Counter for repetitions after unsuccessful attempts

LIS_Update_Repeat:
	rcall I2C_SendStart

	; Establish writing connection
	ldi r16, LIS_ADDRESS << 1 | 0
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp NACK

	; Reading multiple bytes starting from LIS_X_L address
	ldi r16, (1 << 7) | LIS_X_L
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp NACK

	; Restart to start reading
	rcall I2C_SendStart

	; Establish reading connection
	ldi r16, LIS_ADDRESS << 1 | 1
	rcall I2C_Trasmit	
	sbrc r16, 0
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
	rjmp LIS_Update_Exit	; Successful attempt, exit

NACK:
	rcall I2C_SendStop

	pop r16		; Increasing amount of repetitions
	inc r16
	push r16

	cpi r16, 3				; Exit if >= 3
	brge LIS_Update_Exit

	rjmp LIS_Update_Repeat

LIS_Update_Exit:
	pop r16
	ret


; --------------------------------------------
; LIS_TestPrint
; Send X, Y and Z acceleration values to a UART
;
; IN:	X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
;		all variables should be presented in the DSEG
;
; OUT:	None
; --------------------------------------------
LIS_TestPrint:
	lds r16, X_MSB
	rcall abs_print
	rcall Bin2BSD_8
	rcall PrintBSD8

	; ldi r19, ' '
	; rcall Print

	; lds r16, X_LSB
	; rcall Bin2BSD_8
	; rcall PrintBSD8
			
	ldi r19, '\t'
	rcall Print
	ldi r19, '\t'
	rcall Print


	lds r16, Y_MSB
	rcall abs_print
	rcall Bin2BSD_8
	rcall PrintBSD8

	; ldi r19, ' '
	; rcall Print

	; lds r16, Y_LSB
	; rcall Bin2BSD_8
	; rcall PrintBSD8
			
	ldi r19, '\t'
	rcall Print
	ldi r19, '\t'
	rcall Print


	lds r16, Z_MSB
	rcall abs_print
	rcall Bin2BSD_8
	rcall PrintBSD8

	; ldi r19, ' '
	; rcall Print

	; lds r16, Z_LSB
	; rcall Bin2BSD_8
	; rcall PrintBSD8

	ldi r19, 13
	rcall Print

	ret

abs_print:
	tst r16
	brpl abs_end

	subi r16, 1
	ser r17
	eor r16, r17

	push r16
	ldi r19, '-'
	rcall Print
	pop r16

abs_end:
	ret


; --------------------------------------------
; LIS_PrintMemory
; Send whole memory to an UART starting from CTRL_REG1
;
; IN:	None
; OUT:	None
; --------------------------------------------
LIS_PrintMemory:
	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 0
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp LIS_PrintMemory_NACK

	ldi r16, (1 << 7) | 0x20
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp LIS_PrintMemory_NACK

	rcall I2C_SendStart

	ldi r16, LIS_ADDRESS << 1 | 1
	rcall I2C_Trasmit
	sbrc r16, 0
	rjmp LIS_PrintMemory_NACK

	clr r25 ; Counter for printed registers

Read_Loop:
	ldi r16, 0x0
	rcall I2C_Receive

	rcall Bin2BSD_8
	rcall PrintBSD8

	ldi r19, 13
	rcall Print

	inc r25
	cpi r25, 24
	brne Read_Loop	; If read the one before the last

	ldi r16, 0x1
	rcall I2C_Receive

	rjmp LIS_PrintMemory_Exit

LIS_PrintMemory_NACK:
	mprintln "LIS_PrintMemory NACK"

LIS_PrintMemory_Exit:
	rcall I2C_SendStop
	ret


; --------------------------------------------
; LIS_Init
; Init MEMS to work at Normal power mode with x, y and z axes enabled.
; BDU (block data update) enabled. Big endian for acceleration values selected.
;
; IN:	None
; Out:	None
; --------------------------------------------
LIS_Init:
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
	ret


Backup_XYZ:
	lds r16, X_MSB
	sts X_MSB_prev, r16

	lds r16, Y_MSB
	sts Y_MSB_prev, r16

	lds r16, Y_MSB
	sts Y_MSB_prev, r16

	ret


; End Procedure ===========================================


; EEPROM =====================================================
	.ESEG


