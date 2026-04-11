// __lshrhi3 — 16-bit logical shift right for MC6809
//
// hu_int __lshrhi3(hu_int a, int b)
//
// Calling convention: same as __ashrhi3 (see ashrhi3.S).
//
// Unlike __ashrhi3, uses LSR (not ASR) — shifts in 0, not sign bit.

	.section .text,"ax",@progbits
	.globl __lshrhi3
	.type  __lshrhi3,@function
__lshrhi3:
	pshs	x
	andb	#15
	beq	.Llshrhi3_done
.Llshrhi3_loop:
	lsr	,s		// MSB — shift in 0
	ror	1,s		// LSB — through carry from above
	decb
	bne	.Llshrhi3_loop
.Llshrhi3_done:
	puls	x
	rts
	.size __lshrhi3, . - __lshrhi3
