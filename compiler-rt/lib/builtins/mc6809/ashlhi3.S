// __ashlhi3 — 16-bit shift left for MC6809
//
// hi_int __ashlhi3(hi_int a, int b)
//
// Calling convention: same as __ashrhi3 (see ashrhi3.S).
//
// For left shift, shift the LSB first to set carry, then rotate
// the MSB left through carry.

	.section .text,"ax",@progbits
	.globl __ashlhi3
	.type  __ashlhi3,@function
__ashlhi3:
	pshs	x
	andb	#15
	beq	.Lashlhi3_done
.Lashlhi3_loop:
	asl	1,s		// LSB — shift in 0
	rol	,s		// MSB — through carry from above
	decb
	bne	.Lashlhi3_loop
.Lashlhi3_done:
	puls	x
	rts
	.size __ashlhi3, . - __ashlhi3
