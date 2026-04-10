// __lshrsi3 — 32-bit logical shift right for MC6809
//
// su_int __lshrsi3(su_int a, int b)
//
// Calling convention: same as __ashrsi3 (see ashrsi3.S).
//
// Unlike __ashrsi3, uses LSR (not ASR) — shifts in 0, not sign bit.

	.section .text,"ax",@progbits
	.globl __lshrsi3
	.type  __lshrsi3,@function
__lshrsi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldb	7,s
	andb	#31
	beq	.Llshrsi3_done
.Llshrsi3_loop:
	lsr	,x		// byte 0 (MSB) — shift in 0
	ror	1,x
	ror	2,x
	ror	3,x		// byte 3 (LSB)
	decb
	bne	.Llshrsi3_loop
.Llshrsi3_done:
	rts
	.size __lshrsi3, . - __lshrsi3
