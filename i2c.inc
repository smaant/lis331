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