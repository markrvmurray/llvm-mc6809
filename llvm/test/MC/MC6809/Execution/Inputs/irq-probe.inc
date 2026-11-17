;; Probes for codegen-interrupt.s: put known values in the direct-page
;; registers the compiler allocates, take an interrupt, and report whether the
;; handler left them alone. Each returns 0 in X when everything survives, and
;; otherwise a number for the first thing that did not: 1-4 for __rs0-__rs3,
;; 5 for one of __rc0-__rc15, and from the FIRQ probe 6-9 for D, X, Y, U and
;; 10 for W.

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
	ldd	#0x5051
	std	__rc0
	ldd	#0x5253
	std	__rc2
	ldd	#0x5455
	std	__rc4
	ldd	#0x5657
	std	__rc6
	ldd	#0x5859
	std	__rc8
	ldd	#0x5a5b
	std	__rc10
	ldd	#0x5c5d
	std	__rc12
	ldd	#0x5e5f
	std	__rc14
	.endm

	.macro	rc_pair sym, value
	ldd	\sym
	cmpd	#\value
	lbne	1f
	.endm

rs_check:
	ldx	#1
	ldd	__rs0
	cmpd	#0x1111
	lbne	1f
	ldx	#2
	ldd	__rs1
	cmpd	#0x2222
	lbne	1f
	ldx	#3
	ldd	__rs2
	cmpd	#0x3333
	lbne	1f
	ldx	#4
	ldd	__rs3
	cmpd	#0x4444
	lbne	1f
	ldx	#5
	rc_pair	__rc0, 0x5051
	rc_pair	__rc2, 0x5253
	rc_pair	__rc4, 0x5455
	rc_pair	__rc6, 0x5657
	rc_pair	__rc8, 0x5859
	rc_pair	__rc10, 0x5a5b
	rc_pair	__rc12, 0x5c5d
	rc_pair	__rc14, 0x5e5f
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
	lbra	rs_check

	.globl	rs_survive_swi
rs_survive_swi:
	rs_fill
	swi
	lbra	rs_check

;; What any function a handler calls is entitled to do: overwrite every
;; register that is scratch across a call, the direct-page ones included.
;; Handlers that call this must put all of it back.
	.globl	scribble
scribble:
	ldd	#0xeeee
	ldx	#0xeeee
.ifdef __runtime_native_mode
	ldw	#0xeeee
.endif
	std	__rs0
	std	__rs1
	std	__rs2
	std	__rs3
	std	__rc0
	std	__rc2
	std	__rc4
	std	__rc6
	std	__rc8
	std	__rc10
	std	__rc12
	std	__rc14
	rts

;; Nothing in the simulator raises FIRQ, so enter the handler the way the
;; processor would: the return address stacked, then CC with E clear to say
;; the frame is the short one, both interrupts masked, and a jump through the
;; vector. firq_target holds the handler to enter. RTI unstacks CC, sees E
;; clear, and returns here with only the PC.
;;
;; A FIRQ handler is handed nothing saved but PC and CC, so the probe loads
;; every register with a known value first and checks each one afterwards.
	.globl	rs_survive_firq
rs_survive_firq:
	pshs	y,u		; ours to keep for the C caller
	rs_fill
.ifdef __runtime_native_mode
	ldw	#0x6162
.endif
	ldd	#0x7172
	ldx	#0x7374
	ldy	#0x7576
	ldu	#0x7778
	lbsr	2f		; stacks the PC, as the interrupt would
	cmpd	#0x7172
	lbne	6f
	cmpx	#0x7374
	lbne	7f
	cmpy	#0x7576
	lbne	8f
	cmpu	#0x7778
	lbne	9f
.ifdef __runtime_native_mode
	cmpw	#0x6162
	lbne	10f
.endif
	lbsr	rs_check
	puls	y,u,pc
2:	andcc	#0x7f		; E clear: PC and CC are all that is stacked
	pshs	cc
	orcc	#0x50		; FIRQ masks IRQ and itself
	jmp	[firq_target]
6:	ldx	#6
	puls	y,u,pc
7:	ldx	#7
	puls	y,u,pc
8:	ldx	#8
	puls	y,u,pc
9:	ldx	#9
	puls	y,u,pc
10:	ldx	#10
	puls	y,u,pc

	.section .bss
	.globl	firq_target
firq_target:
	.space	2
