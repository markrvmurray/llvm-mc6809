// __lshrdi3 — 64-bit logical shift right for MC6809
//
// du_int __lshrdi3(du_int a, int b)
//
// Calling convention: same as __ashrdi3 (see ashrdi3.S).
//
// Algorithm: copy input to result buffer, then loop LSR/ROR
// from MSB to LSB for each bit of shift count.
// Unlike __ashrdi3, uses LSR (not ASR) — shifts in 0, not sign bit.

	.section .text,"ax",@progbits
	.globl __lshrdi3
	.type  __lshrdi3,@function
__lshrdi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
	ldb	11,s
	andb	#63
	beq	.Llshrdi3_done
.Llshrdi3_loop:
	lsr	,x		// byte 0 (MSB) — shift in 0
	ror	1,x
	ror	2,x
	ror	3,x
	ror	4,x
	ror	5,x
	ror	6,x
	ror	7,x		// byte 7 (LSB)
	decb
	bne	.Llshrdi3_loop
.Llshrdi3_done:
	rts
	.size __lshrdi3, . - __lshrdi3
