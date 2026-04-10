// __ashrsi3 — 32-bit arithmetic shift right for MC6809
//
// si_int __ashrsi3(si_int a, int b)
//
// Calling convention (gcc6809 ABI, sret for i32 return):
//   X  = pointer to caller-allocated 4-byte result buffer
//   Stack (after return address):
//     S+2..S+5   i32 input 'a' (big-endian, MSB at S+2)
//     S+6..S+7   i16 shift count 'b' (only low 5 bits used)
//
// The shift amount is declared as i16 in shiftRotateLibcall()
// (matching the C `int` type) so it lands on the stack — same
// reasoning as __ashrdi3: B may get clobbered by the i32 arg
// stack setup.
//
// Algorithm: copy input to result buffer, then loop ASR/ROR
// through all 4 bytes for each bit of shift count.
//
// Registers used: A, B, X (all caller-saved). Y and U untouched.

	.section .text,"ax",@progbits
	.globl __ashrsi3
	.type  __ashrsi3,@function
__ashrsi3:
	// Copy 4-byte input from stack to result buffer at X.
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	// Load shift count from stack (low byte of i16, masked to 0-31).
	ldb	7,s
	andb	#31
	beq	.Lashrsi3_done
.Lashrsi3_loop:
	asr	,x		// byte 0 (MSB) — sign bit propagates
	ror	1,x
	ror	2,x
	ror	3,x		// byte 3 (LSB)
	decb
	bne	.Lashrsi3_loop
.Lashrsi3_done:
	rts
	.size __ashrsi3, . - __ashrsi3
