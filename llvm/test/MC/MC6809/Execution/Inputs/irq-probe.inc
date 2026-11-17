;; Probes for codegen-interrupt.s: put known values in the four direct-page
;; registers the compiler allocates, take an interrupt, and report whether the
;; handler left them alone. Each returns 0 in X when all four survive, and
;; otherwise the number (1-4) of the first that did not.

	.section .rom,"ax",@progbits

;; Let IRQs through. With __runtime_native_mode defined (a 6309 run) the
;; processor is switched to native mode first, which is the only 6309 mode a
;; handler supports: there an interrupt stacks W along with the rest.
	.globl	irq_enable
irq_enable:
.ifdef __runtime_native_mode
	ldmd	#1
.endif
	andcc	#0xef
	rts

	.macro	rs_fill
	ldd	#0x1111
	std	__rs0
	ldd	#0x2222
	std	__rs1
	ldd	#0x3333
	std	__rs2
	ldd	#0x4444
	std	__rs3
	.endm

rs_check:
	ldx	#1
	ldd	__rs0
	cmpd	#0x1111
	bne	1f
	ldx	#2
	ldd	__rs1
	cmpd	#0x2222
	bne	1f
	ldx	#3
	ldd	__rs2
	cmpd	#0x3333
	bne	1f
	ldx	#4
	ldd	__rs3
	cmpd	#0x4444
	bne	1f
	ldx	#0
1:	rts

;; The ACIA requests an interrupt as soon as its transmit interrupt is
;; enabled, since the transmit register is empty. The handler turns it off.
	.globl	rs_survive_irq
rs_survive_irq:
	rs_fill
	lda	#0x35		; as _acia_init, with the transmit interrupt on
	sta	0xffc4
	nop
	nop
	bra	rs_check

	.globl	rs_survive_swi
rs_survive_swi:
	rs_fill
	swi
	bra	rs_check
