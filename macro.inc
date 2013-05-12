.macro outi          	
	ldi    r16, @1

	.if @0 < 0x40
		out @0, r16       
	.else
		sts @0, r16
	.endif
	.endm


.macro outu
	.if	@0 < 0x40
		out	@0, @1         
	.else
		sts	@0, @1
	.endif
.endm


.macro setb
	.if @0 < 0x20
		sbi @0, @1

	.else
		.if @0 < 0x40
			in @2, @0
			ori @2, 1<<@1
			out @0, @2

		.else
			lds @2, @0
			ori @2, 1<<@1
			sts @0, @2
		.endif
	.endif
.endm


;SET Bit with stack
.macro	setbm 
	.if @0 < 0x20
		sbi	@0, @1

	.else
		.if @0 < 0x40
			push r17
			in	 r17, @0
			ori	 r17, 1<<@1
			out	 @0,  r17
			pop	 r17

		.else
			push r17
			lds	 r17, @0
			ori	 r17, 1<<@1
			sts	 @0,  r17
			pop	 r17
		.endif
	.endif
.endm


.macro	clrb
	.if @0 < 0x20
		cbi	@0, @1

	.else
		.if @0<0x40
			in   @2, @0
			andi @2, ~(1<<@1)
			out	 @0, @2

		.else
			lds	 @2, @0
			andi @2, ~(1<<@1)
			sts	 @0, @2
		.endif
	.endif
.endm



; clrbm byte, bit -- cleare given bit in given byte, use stack for temp variables
.macro	clrbm 
	.if @0 < 0x20
		cbi	@0, @1

	.else
		.if @0<0x40
			push r17
			in   r17, @0
			andi r17, ~(1<<@1)
			out	 @0,  r17
			pop	 r17

		.else
			push r17
			lds	 r17, @0
			andi r17, ~(1<<@1)
			sts	 @0,  r17
			pop	 r17
		.endif
	.endif
.endm


; invbm byte, bit -- invert given bit in given byte using stack for temp variables
.macro invbm
	.if	@0 < 0x40
		push r16
		push r17
		in   r16, @0
		ldi  r17, 1<<@1
		eor  r17, r16
		out  @0,  r17
		pop  r17
		pop  r16

	.else
		push r16
		push r17
		lds  r16, @0
		ldi  r17, 1<<@1
		eor  r17, r16
		sts  @0,  r17
		pop  r17
		pop  r16
	.endif
.endm


.macro incm
	lds	r16, @0
	lds	r17, @0+1
	lds	r18, @0+2
	lds	r19, @0+3

	subi r16, (-1)
	sbci r17, (-1)
	sbci r18, (-1)
	sbci r19, (-1)

	sts @0, r16
	sts	@0+1, r17
	sts	@0+2, r18
	sts	@0+3, r19
.endm


.macro pushf
	push r16
	in   r16, SREG
	push r16
.endm


.macro popf
	pop r16
	out SREG, r16
	pop r16
.endm
