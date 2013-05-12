.DSEG

	; UART buffers
	.equ MAXBUFF_IN  = 10
	.equ MAXBUFF_OUT = 10

	IN_buff:  .byte MAXBUFF_IN
	IN_ptr_s: .byte 1
	IN_ptr_e: .byte 1
	IN_full:  .byte 1

	OUT_buff:  .byte MAXBUFF_OUT
	OUT_ptr_s: .byte 1
	OUT_ptr_e: .byte 1
	OUT_full:  .byte 1


.CSEG

; =========================================================
; UART Interruption handlers
; =========================================================

; RX Complete Intrruption (RXCIE)
RX_OK_Int:
	pushf
	push xl
	push xh
	push r17
	push r18

	ldi xl, low(IN_buff)
	ldi xh, high(IN_buff)
	lds r16, IN_ptr_e
	lds r18, IN_ptr_s

	add xl, r16
	clr r17
	adc xh, r17

	in r17, UDR
	st x, r17

	inc r16

	cpi r16, MAXBUFF_IN
	brne RX_OK_NoEnd
	clr r16

RX_OK_NoEnd:
	cp r16, r18
	brne RX_OK_Exit

RX_OK_OVERFLOW:
	outi IN_full, 1

RX_OK_Exit:
	sts IN_ptr_e, r16

	pop r18
	pop r17
	pop xh
	pop xl
	popf
	ret


; TX Complete Interruption (TXCIE)
TX_OK_Int:
	ret


; UDR Empty Interruption (UDRIE)
UDR_Emp_Int:
	pushf
	push xl
	push xh
	push r17
	push r18
	push r19

	ldi xl, low(OUT_buff)
	ldi xh, high(OUT_buff)
	lds r16, OUT_ptr_e
	lds r18, OUT_ptr_s
	lds r19, OUT_full

	cpi r19, 1
	breq UDR_Emp_Send

	cp r16, r18
	brne UDR_Emp_Send

	clrb UCSRB, UDRIE, r17
	rjmp UDR_Emp_Exit

UDR_Emp_Send:
	outi OUT_full, 0

	add xl, r18
	clr r17
	adc xh, r17

	ld r17, x
	out UDR, r17

	inc r18

	cpi r18, MAXBUFF_OUT
	brne UDR_Emp_Exit

	clr r18

UDR_Emp_Exit:
	sts OUT_ptr_s, r18

	pop r19
	pop r18
	pop r17
	pop xh
	pop xl
	popf
	ret

; =========================================================
; UART functions
; =========================================================

; --------------------------------------------
; UART_Init
;
; IN/OUT: None
; --------------------------------------------
UART_Init:
	.equ f_osc = 1000000
	.equ baudrate = 9600
	.equ uart2x = 1 		; 1 if U2X required
	.equ bauddivider = f_osc / (16 / (uart2x + 1) * baudrate) - 1

	outi UBRRL, low(bauddivider)
	outi UBRRH, high(bauddivider)

	outi UCSRA, 0

	; Enable U2X if required 
	outi UCSRA, (uart2x<<U2X)

	; Enable TX/RX and Interaptions (except UDRIE)
	outi UCSRB, (1<<RXEN) | (1<<TXEN) | (1<<RXCIE) | (1<<TXCIE) | (0<<UDRIE)

	; 8 bit character size, 1 stop bit
	outi UCSRC, (0<<USBS) | (3<<UCSZ0)
	ret

; --------------------------------------------
; ReadBuffer
;
; IN:  None
; OUT: r17 - data
;	   r19 - error code
; --------------------------------------------
ReadBuffer:
	cli

	ldi xl, low(IN_buff)
	ldi xh, high(IN_buff)
	lds r16, IN_ptr_e
	lds r18, IN_ptr_s
	lds r19, IN_full

	cpi r19, 1 		; Overflow?
	breq RB_Read

	cp r16, r18		; Buffer empty (Start == End)?
	brne RB_Read

	ldi r19, 1 		; Sign of empty buffer

	rjmp RB_Exit

RB_Read:
	outi IN_full, 0 ; Clear sign of overflow

	add xl, r18		; Calculate data addres in X by offset (IN_ptr_s)
	clr r17
	adc xh, r17

	ld r17, x 		; Load data
	clr r19			; Clear error code

	inc r18			; Move start pointer

	cpi r18, MAXBUFF_IN ; End of the buffer reached?
	brne RB_Exit

	clr r18			; Move start to the beginning if Yes

RB_Exit:
	sts IN_ptr_s, r18
	sei
	ret


; --------------------------------------------
; WriteBuffer
;
; IN:  r19 - data
; OUT: r19 - error code
; --------------------------------------------
WriteBuffer:
	cli

	ldi xl, low(OUT_buff)
	ldi xh, high(OUT_buff)
	lds r16, OUT_ptr_e
	lds r18, OUT_ptr_s

	add xl, r16
	clr r17
	adc xh, r17

	st x, r19
	clr r19

	inc r16

	cpi r16, MAXBUFF_OUT	; End of the buffer has been reached?
	brne WB_NoEnd

	clr r16					; Yes -- move end to the beginning

WB_NoEnd:
	cp r16, r18				; Overflow (Start == End)?
	brne WB_Exit

WB_OUT_Full:
	ldi r19, 1 				; Will return overflow error
	sts OUT_full, r19		; And set up overflow flag

WB_Exit:
	sts OUT_ptr_e, r16		; Save new end position
	sei
	ret


; --------------------------------------------
; PrintStr
;
; IN:  r16 - low(str)
;	   r17 - high(str)
; OUT: None
; --------------------------------------------
PrintStr:	
	mov zl, r16
	mov zh, r17

PrintStr_loop:
	lpm r19, z+

	cpi r19, 0
	breq PrintStr_Exit

	push zl
	push zh

	rcall Print

	pop zh
	pop zl

	rjmp PrintStr_loop

PrintStr_Exit:
	ret


; --------------------------------------------
; Print
;
; IN:  r19 - data
; OUT: None
; --------------------------------------------
Print:
Print_loop:
	lds r16, OUT_full
	cpi r16, 1
	breq Print_loop

	rcall WriteBuffer
	setb UCSRB, UDRIE, r16

	ret


; --------------------------------------------
; PrintBSD8
;
; IN:  r17 - BSD low
;	   r18 - BSD high
; OUT: None
; --------------------------------------------
PrintBSD8:
	ldi r20, 4  ; Counter of half-bytes in BSD number

	clr r21 	; Flag to skip all zeros till first non zero digit

PrintBSD8_loop:
	clr r19

	lsl r17
	rol r18
	rol r19

	lsl r17
	rol r18
	rol r19

	lsl r17
	rol r18
	rol r19

	lsl r17
	rol r18
	rol r19

	cpi r20, 1
	breq PrintBSD8_last_digit

	mov r22, r21
	add r22, r19
	; cpi r22, 0
	breq PrintBSD8_loop_end

PrintBSD8_last_digit:
	ldi r21, 1

	ldi r16, 0x30
	add r19, r16

	push r17
	push r18
	push r20
	push r21

	rcall Print

	pop r21
	pop r20
	pop r18
	pop r17

PrintBSD8_loop_end:
	dec r20
	breq PrintBSD8_done

	rjmp PrintBSD8_loop

PrintBSD8_done:
	ret


; =========================================================
; Macroses
; =========================================================

.macro mprint
	.if STRLEN("@0") == 3
		ldi r19, @0
		rcall Print
	.else
		lbl:
		.db @0, 0
		ldi r16, low(lbl * 2)
		ldi r17, high(lbl * 2)
		rcall PrintStr
	.endif
.endm


.macro mprintln
	mprint @0

	ldi r19, 13
	rcall Print
.endm
