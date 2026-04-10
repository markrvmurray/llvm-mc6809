// __ashlsi3 — 32-bit shift left for MC6809
//
// si_int __ashlsi3(si_int a, int b)
//
// Calling convention: same as __ashrsi3 (see ashrsi3.S).
//
// Algorithm: copy input to result buffer, then loop ASL/ROL
// from LSB to MSB for each bit of shift count.

	.section .text,"ax",@progbits
	.globl __ashlsi3
	.type  __ashlsi3,@function
__ashlsi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldb	7,s
	andb	#31
	beq	.Lashlsi3_done
.Lashlsi3_loop:
	asl	3,x		// byte 3 (LSB) — shift in 0
	rol	2,x
	rol	1,x
	rol	,x		// byte 0 (MSB)
	decb
	bne	.Lashlsi3_loop
.Lashlsi3_done:
	rts
	.size __ashlsi3, . - __ashlsi3
