// __ashrdi3 — 64-bit arithmetic shift right for MC6809
//
// di_int __ashrdi3(di_int a, int b)
//
// Calling convention (gcc6809 ABI, sret for i64 return):
//   X  = pointer to caller-allocated 8-byte result buffer
//   Stack (after return address):
//     S+2..S+9   i64 input 'a' (big-endian, MSB at S+2)
//     S+10..S+11 i16 shift count 'b' (only low 6 bits used)
//
// The shift amount is declared as i16 in shiftRotateLibcall()
// (matching the C `int` type) so it lands on the stack. Using i8
// (which would go in B) is unreliable because B gets clobbered by
// the i64 arg stack setup code.
//
// Algorithm: copy input to result buffer, then loop ASR/ROR
// through all 8 bytes for each bit of shift count.
//
// Registers used: A, B, X (all caller-saved). Y and U untouched.
// Code size: 38 bytes. Worst case (shift by 63): ~4000 cycles.

	.section .text,"ax",@progbits
	.globl __ashrdi3
	.type  __ashrdi3,@function
__ashrdi3:
	// Copy 8-byte input from stack to result buffer at X.
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
	// Load shift count from stack (low byte, masked to 0-63).
	ldb	11,s
	andb	#63
	beq	.Lashrdi3_done
.Lashrdi3_loop:
	asr	,x		// byte 0 (MSB) — sign bit propagates
	ror	1,x
	ror	2,x
	ror	3,x
	ror	4,x
	ror	5,x
	ror	6,x
	ror	7,x		// byte 7 (LSB)
	decb
	bne	.Lashrdi3_loop
.Lashrdi3_done:
	rts
	.size __ashrdi3, . - __ashrdi3
