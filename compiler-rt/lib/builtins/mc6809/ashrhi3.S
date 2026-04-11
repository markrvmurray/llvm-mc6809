// __ashrhi3 — 16-bit arithmetic shift right for MC6809
//
// hi_int __ashrhi3(hi_int a, int b)
//
// Calling convention (gcc6809 ABI, ≤2-byte return):
//   IX = i16 input 'a' (also holds the i16 return value)
//   B  = i8 shift count 'b' (only low 4 bits used)
//
// The legalizer's shiftRotateLibcall declares the amount as i8 for
// sizes ≤ 32, so it lands in B. IX holds 'a' as both input and
// output (i16 returns go in IX per the gcc6809 ABI). B is independent
// of IX so there's no clobber risk during arg setup.
//
// Algorithm: spill X to a 2-byte stack scratch, shift the bytes
// in memory, reload X. Avoids the LSRA/RORB register chain that
// would tie up D, which is the entire reason this libcall exists.
//
// Registers used: B, X (caller-saved). A, Y, U untouched.
// Stack usage: 2 bytes (one PSHS X / PULS X pair).

	.section .text,"ax",@progbits
	.globl __ashrhi3
	.type  __ashrhi3,@function
__ashrhi3:
	pshs	x		// S+0 = X.hi (MSB), S+1 = X.lo (LSB)
	andb	#15
	beq	.Lashrhi3_done
.Lashrhi3_loop:
	asr	,s		// MSB — sign bit propagates
	ror	1,s		// LSB — through carry from above
	decb
	bne	.Lashrhi3_loop
.Lashrhi3_done:
	puls	x
	rts
	.size __ashrhi3, . - __ashrhi3
