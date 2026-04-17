// __ashldi3 — 64-bit shift left for MC6809
//
// di_int __ashldi3(di_int a, int b)
//
// Calling convention: same as __ashrdi3 (see ashrdi3.S).
//
// Algorithm: copy input to result buffer, then loop ASL/ROL
// from LSB to MSB for each bit of shift count.

	.section .text,"ax",@progbits
	.globl __ashldi3
	.type  __ashldi3,@function
__ashldi3:
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
	beq	.Lashldi3_done
.Lashldi3_loop:
	asl	7,x		// byte 7 (LSB) — shift in 0
	rol	6,x
	rol	5,x
	rol	4,x
	rol	3,x
	rol	2,x
	rol	1,x
	rol	,x		// byte 0 (MSB)
	decb
	bne	.Lashldi3_loop
.Lashldi3_done:
	rts
	.size __ashldi3, . - __ashldi3
