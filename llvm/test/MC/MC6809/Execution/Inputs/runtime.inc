;;
;; runtime.inc — shared test runtime for MC6809 execution tests
;;
;; Memory map (matches usim09batch):
;;   0x0000-0xbfff  RAM (48KB)
;;   0xc000         ACIA status register (overlays ROM)
;;   0xc001         ACIA data register
;;   0xc002         Halt/exit device (written value = exit code)
;;   0xc003-0xffef  ROM (code)
;;   0xfff0-0xffff  Vectors
;;

;; ===== Entry point =====

	.section .rom,"ax",@progbits
	.globl	_start
_start:
	lds	#0xc000		; stack at top of RAM (48KB)
	clra
	tfr	a,dp		; DP=0 (direct page at $0000)

	;; Initialise MC6850 ACIA (required by MAME; USim ignores this).
	lda	#0x03		; master reset
	sta	0xc000
	lda	#0x15		; /16, 8N1, no IRQ
	sta	0xc000

	;; Copy .data initial values from ROM (LMA) to RAM (VMA).
	ldx	#__data_load	; ROM source
	ldy	#__data_start	; RAM dest
.Ldata_copy:
	cmpy	#__data_end
	beq	.Ldata_done
	lda	,x+
	sta	,y+
	bra	.Ldata_copy
.Ldata_done:

	;; Zero .bss
	ldx	#__bss_start
.Lbss_zero:
	cmpx	#__bss_end
	beq	.Lbss_done
	clr	,x+
	bra	.Lbss_zero
.Lbss_done:

	jsr	test_main
	jsr	halt
_hang:
	bra	_hang

;; ===== putchar =====
;; Write the byte in A to the ACIA.
;; Preserves all registers.

putchar:
	pshs	b
.Lputchar_wait:
	ldb	0xc000		; ACIA status
	bitb	#0x02		; TDRE
	beq	.Lputchar_wait
	sta	0xc001		; ACIA data
	puls	b,pc

;; ===== puthex =====
;; Write the byte in A as two hex digits to the ACIA.
;; Preserves all registers.

puthex:
	pshs	a
	lsra
	lsra
	lsra
	lsra
	bsr	.Lprint_nibble
	puls	a
	pshs	a
	anda	#0x0f
	bsr	.Lprint_nibble
	puls	a,pc

.Lprint_nibble:
	cmpa	#10
	blo	.Ldigit
	adda	#7		; 'A' - 10 - '0'
.Ldigit:
	adda	#0x30		; '0'
	bra	putchar

;; ===== putstr =====
;; Write a null-terminated string pointed to by X to the ACIA.
;; Advances X past the string. Preserves other registers.

putstr:
	pshs	a
.Lputstr_loop:
	lda	,x+
	beq	.Lputstr_done
	bsr	putchar
	bra	.Lputstr_loop
.Lputstr_done:
	puls	a,pc

;; ===== putnl =====
;; Write a newline (LF) to the ACIA.
;; Preserves all registers.

putnl:
	pshs	a
	lda	#0x0a
	bsr	putchar
	puls	a,pc

;; ===== halt =====
;; Halt the CPU by writing to the halt device.
;; The written byte becomes the process exit code.

halt:
	clra
	sta	0xc002
	rts

;; ===== Vector table =====

	.section .vectors,"a",@progbits
	.word	0		; 0xfff0: reserved
	.word	0		; 0xfff2: SWI3
	.word	0		; 0xfff4: SWI2
	.word	0		; 0xfff6: FIRQ
	.word	0		; 0xfff8: IRQ
	.word	0		; 0xfffa: SWI
	.word	0		; 0xfffc: NMI
	.word	_start		; 0xfffe: RESET
