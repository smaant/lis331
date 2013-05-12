

.macro RAM_Flush
	; Clear RAM
	ldi	zl, Low(SRAM_START)
	ldi	zh, High(SRAM_START)
	clr	r16
	
Flush:
	st z+, r16				; set 0 to memory adress
	cpi zh, High(RAMEND+1)	; is it the end of a RAM for High part of address?
	brne Flush				; continue if No

	cpi zl, Low(RAMEND+1)	; is it the end of a RAM for Low part of address?
	brne Flush

	; Clear Registrers
	ldi zl, 30		; we have 32 registers and we'll skip Z (R30-31) now
	clr zh			; set ZH = 0 to use it as 'cleaner'
	dec zl
	st  z, zh		; clean *Z register
	brne PC-2		; go to the decrement line

	clr zl			; clean last uncleaned register

	; Init Stack
	ldi r16, Low(RAMEND)
	out spl, r16

	; ldi r16, High(RAMEND)
	; out SPH, r16
.endm
