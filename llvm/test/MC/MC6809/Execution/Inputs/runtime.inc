;;
;; runtime.inc — shared test runtime for MC6809 execution tests
;;
;; Memory map (matches usim09batch as of usim commit 78a1fa4,
;; "pico-thing alignment" — Bug #358; the IO slab moved down from
;; $FFD0-$FFD2 to $FFC4-$FFCC):
;;   0x0000-0xffc3  RAM (single unified region)
;;   0xffc4         ACIA status register (bit 1 = TDRE, bit 0 = RDRF)
;;   0xffc5         ACIA data register
;;   0xffcb         TraceCtl (USim --brk-gated / --trace toggle)
;;   0xffcc         Halt/exit device (written value = exit code)
;;   0xfff0-0xffff  Vectors (vector ROM)
;;

;; ===== Entry point =====
;;
;; Wrapped in `.ifndef __runtime_skip_start` so a sibling include like
;; `runtime6309.inc` can replace the prologue (e.g. to inject `LDMD #1`
;; for HD6309 native mode) without text-duplicating the helpers below.
;; When `__runtime_skip_start` is undefined, `_start` is emitted as
;; before — bytes-for-bytes identical to the pre-#194 layout.

.ifndef __runtime_skip_start

	.section .rom,"ax",@progbits
	.globl	_start
_start:
	;; Stack at top of DRAM (data region). Bug #181: previous value
	;; was $C000, matching the old DRAM=$0000-$BFFF / CRAM=$C003-$FFCF
	;; split. After the split moved to $0000-$3FFF / $4000-$FFCF,
	;; the stack starts at $4000 and grows down into DRAM.
	lds	#0x4000		; stack at top of low-RAM data region
	clra
	tfr	a,dp		; DP=0 (direct page at $0000)

	;; Zero .bss
	ldx	#__bss_start
.Lbss_zero:
	cmpx	#__bss_end
	beq	.Lbss_done
	clr	,x+
	bra	.Lbss_zero
.Lbss_done:

	jsr	_acia_init
	jsr	test_main
	jsr	halt
_hang:
	bra	_hang

.endif

;; ===== _acia_init =====
;; Initialise the MC6850 ACIA so that putchar's TDRE poll succeeds.
;; usim09batch's emulated ACIA asserts TDRE from boot regardless, so
;; this is a no-op on usim. MAME's HD6309 SBC (`mamellvm6309`) requires
;; an explicit master-reset ($03 → control) followed by an operating-
;; mode write ($15 = /16 clock, 8N1, no IRQ, RTS=low) before TDRE will
;; assert; without this, every putchar polls $FFC4 forever and the
;; test hits its runaway timeout.
;; Called unconditionally by _start so every test that calls into the
;; runtime is portable across both simulators.
;; Preserves all registers.

_acia_init:
	pshs	a
	lda	#0x03		; master reset
	sta	0xffc4
	lda	#0x15		; /16 clock, 8N1, no IRQ, RTS=low
	sta	0xffc4
	puls	a,pc

;; ===== putchar =====
;; Write the byte in A to the ACIA.
;; Preserves all registers.

putchar:
	pshs	b
.Lputchar_wait:
	ldb	0xffc4		; ACIA status
	bitb	#0x02		; TDRE
	beq	.Lputchar_wait
	sta	0xffc5		; ACIA data
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
	sta	0xffcc
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
