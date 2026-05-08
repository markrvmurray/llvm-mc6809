//===-- divsi.S - 32-bit division for MC6809 ---------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// 32-bit signed and unsigned division and remainder.
//
// gcc6809 ABI v1 (sret-style):
//   IX        = sret pointer (caller-allocated 4-byte buffer)
//   2,S..3,S  = a_hi (dividend high half)
//   4,S..5,S  = a_lo (dividend low half)
//   6,S..7,S  = b_hi (divisor high half)
//   8,S..9,S  = b_lo (divisor low half)
// Result is written through IX (hi at IX+0, lo at IX+2; big-endian).
// Clobbers: D. Preserves: Y, U.
//
// Algorithm: restoring division (shift-and-subtract), 32 iterations.
//
// HD6309 (`__6309__`): unsigned variants take a fast path when the
// divisor's high half is zero AND the low half is in the signed-DIVQ-
// safe range [2, 0x7FFF]. Long-division by a 16-bit divisor uses two
// DIVQ instructions: ~80 cycles vs ~600 for the 32-iteration loop,
// ~7-8x speedup. Bug #161 Phase 3.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

// ============================================================
// __udivsi3 -- unsigned 32-bit division (returns quotient)
// ============================================================
	.globl	__udivsi3
	.type	__udivsi3,@function
__udivsi3:
#ifdef __6309__
	// HD6309 fast path test (pre-mutation offsets):
	//   0..1=ret, 2..3=a_hi, 4..5=a_lo, 6..7=b_hi, 8..9=b_lo
	ldd	6,s		// b_hi == 0?
	bne	.Lud32_slow	// no → 6809 loop
	ldd	8,s		// b_lo
	beq	.Lud32_slow	// b_lo == 0 → loop (undefined behavior)
	bmi	.Lud32_slow	// b_lo high bit set → loop (signed-DIVQ safety)
	cmpd	#1
	beq	.Lud32_div_by_1
	// Fast path. b_lo ∈ [2, 0x7FFF].
	// Phase 1: Q = (0 : a_hi); DIVQ b_lo → W = q_hi, D = remainder
	ldd	#0
	ldw	2,s		// W = a_hi
	divq	8,s		// → W = q_hi, D = remainder ∈ [0, b_lo-1]
	pshsw			// save q_hi (W) — note: 'PSHS W' is invalid;
				//   HD6309 W needs the dedicated PSHSW opcode
	// S: 0..1=q_hi, 2..3=ret, 4..5=a_hi, 6..7=a_lo, 8..9=b_hi, 10..11=b_lo
	ldw	6,s		// W = a_lo (Phase 2 input; D still has remainder)
	divq	10,s		// Q = (rem:a_lo) ÷ b_lo → W = q_lo, D = final rem
	ldd	,s		// D = q_hi
	std	,x		// *X (high 16) = q_hi
	stw	2,x		// *(X+2) (low 16) = q_lo
	leas	2,s		// pop saved q_hi
	rts

.Lud32_div_by_1:
	// quotient = dividend; copy a → *X
	ldd	2,s		// a_hi
	std	,x
	ldd	4,s		// a_lo
	std	2,x
	rts

.Lud32_slow:
#endif
	pshs	y		// preserve callee-saved Y
	leas	-9,s		// workspace: quot(4) + rem(4) + counter(1)
	// Frame layout (relative to S):
	//   S+0..3   = quotient workspace
	//   S+4..7   = remainder workspace
	//   S+8      = counter
	//   S+9..10  = saved Y
	//   S+11..12 = return addr
	//   S+13..14 = a_hi
	//   S+15..16 = a_lo
	//   S+17..18 = b_hi
	//   S+19..20 = b_lo

	// Init quotient = a (dividend)
	ldd	13,s
	std	,s
	ldd	15,s
	std	2,s

	// Init remainder = 0
	ldd	#0
	std	4,s
	std	6,s

	// Counter = 32
	lda	#32
	sta	8,s

.Lud32_loop:
	// Shift quotient left (32-bit)
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s		// MSB -> carry

	// Rotate carry into remainder (32-bit)
	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	// Compare remainder with divisor (32-bit unsigned)
	ldd	4,s		// remainder hi
	cmpd	17,s		// vs divisor hi
	bhi	.Lud32_sub
	blo	.Lud32_skip
	ldd	6,s		// remainder lo
	cmpd	19,s		// vs divisor lo
	blo	.Lud32_skip

.Lud32_sub:
	ldd	6,s		// remainder lo
	subd	19,s		// - divisor lo
	std	6,s
	lda	5,s
	sbca	18,s
	sta	5,s
	lda	4,s
	sbca	17,s
	sta	4,s

	// Set quotient bit 0
	lda	3,s
	ora	#1
	sta	3,s

.Lud32_skip:
	dec	8,s
	bne	.Lud32_loop

	// Copy quotient to caller's sret buffer
	ldd	,s
	std	,x
	ldd	2,s
	std	2,x

	leas	9,s
	puls	y
	rts

// ============================================================
// __umodsi3 -- unsigned 32-bit remainder
// ============================================================
	.globl	__umodsi3
	.type	__umodsi3,@function
__umodsi3:
#ifdef __6309__
	ldd	6,s		// b_hi == 0?
	bne	.Lum32_slow
	ldd	8,s
	beq	.Lum32_slow	// b_lo == 0 → loop
	bmi	.Lum32_slow	// b_lo ≥ 0x8000 → loop
	cmpd	#1
	beq	.Lum32_div_by_1
	// Fast path. Same two-DIVQ scheme; we only need the final
	// 16-bit remainder (the high 16 of the 32-bit remainder is
	// always zero because b_lo ≤ 0x7FFF means remainder ≤ 0x7FFE).
	ldd	#0
	ldw	2,s		// W = a_hi
	divq	8,s		// → W = q_hi (discard), D = remainder
	ldw	4,s		// W = a_lo (Phase 2 input)
	divq	8,s		// → W = q_lo (discard), D = final remainder
	pshs	d		// save remainder
	ldd	#0
	std	,x		// *X (high 16) = 0
	puls	d
	std	2,x		// *(X+2) (low 16) = remainder
	rts

.Lum32_div_by_1:
	// remainder = 0; write zeros to *X
	ldd	#0
	std	,x
	std	2,x
	rts

.Lum32_slow:
#endif
	pshs	y
	leas	-9,s

	ldd	13,s
	std	,s
	ldd	15,s
	std	2,s

	ldd	#0
	std	4,s
	std	6,s

	lda	#32
	sta	8,s

.Lum32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	17,s
	bhi	.Lum32_sub
	blo	.Lum32_skip
	ldd	6,s
	cmpd	19,s
	blo	.Lum32_skip

.Lum32_sub:
	ldd	6,s
	subd	19,s
	std	6,s
	lda	5,s
	sbca	18,s
	sta	5,s
	lda	4,s
	sbca	17,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lum32_skip:
	dec	8,s
	bne	.Lum32_loop

	// Copy remainder to caller's sret buffer
	ldd	4,s
	std	,x
	ldd	6,s
	std	2,x

	leas	9,s
	puls	y
	rts

// ============================================================
// __divsi3 -- signed 32-bit division (returns quotient)
// Result sign = sign(a) XOR sign(b).
// ============================================================
	.globl	__divsi3
	.type	__divsi3,@function
__divsi3:
#ifdef __6309__
	// HD6309 fast path: |b|.hi == 0 AND |b|.lo ∈ [2, 0x7FFF].
	// Mirror of the unsigned __udivsi3 fast path with abs/sign-fix
	// wrapping (model: divhi.S:177 __divhi3 for 16-bit).
	//
	// Pre-pshs frame:
	//   S+0..1=ret, S+2..3=a_hi, S+4..5=a_lo, S+6..7=b_hi, S+8..9=b_lo
	//
	// Use a local 8-byte workspace (4 bytes |a| + 4 bytes |b|) so we
	// don't trample caller's args before eligibility is confirmed —
	// otherwise jumping to the slow path would feed it modified input.
	pshs	y			// save callee-saved Y
	leas	-8,s			// |a|(4) + |b|(4)
	// Frame:
	//   S+0..3   = |a| (workspace, hi at S+0)
	//   S+4..7   = |b| (workspace, hi at S+4)
	//   S+8..9   = saved Y
	//   S+10..11 = ret
	//   S+12..13 = a_hi
	//   S+14..15 = a_lo
	//   S+16..17 = b_hi
	//   S+18..19 = b_lo

	// Compute |b| in workspace.  Branchless 32-bit two's complement
	// via 0 - value: SUBD on lo sets C=(lo!=0), then SBCD on hi
	// computes 0 - hi - C, which equals -hi when lo==0 and ~hi when
	// lo!=0 — exactly the carry-propagated negate we need.
	ldd	16,s			// b_hi
	std	4,s
	ldd	18,s			// b_lo
	std	6,s
	tst	4,s			// |b|.hi MSB = sign(b_orig)
	bpl	.Lsd32q_b_pos
	ldd	#0
	subd	6,s			// D = -b_lo,  C = (b_lo != 0)
	std	6,s
	ldd	#0
	sbcd	4,s			// D = 0 - b_hi - C
	std	4,s
.Lsd32q_b_pos:

	// Eligibility: |b|.hi == 0 AND |b|.lo ∈ [2, 0x7FFF].
	ldd	4,s			// |b|.hi
	bne	.Lsd32q_slow
	ldd	6,s			// |b|.lo
	beq	.Lsd32q_slow		// |b| == 0 → undefined behaviour, slow
	bmi	.Lsd32q_slow		// |b|.lo ≥ 0x8000 → signed-DIVQ unsafe

	// Compute |a| in workspace.
	ldd	12,s			// a_hi
	std	,s
	ldd	14,s			// a_lo
	std	2,s
	tst	,s
	bpl	.Lsd32q_a_pos
	ldd	#0
	subd	2,s
	std	2,s
	ldd	#0
	sbcd	,s
	std	,s
.Lsd32q_a_pos:

	// Compute sign_q = sign(a_orig) XOR sign(b_orig) → 1-byte flag.
	// Read original a_hi.MSB and b_hi.MSB from the unmodified caller
	// stack slots (we only mutated the workspace, not the args).
	lda	12,s			// a_hi.MSB at S+12
	eora	16,s			// XOR b_hi.MSB at S+16
	pshs	a			// push sign_q byte
	// New offsets after pshs A (everything shifted by +1):
	//   S+0     = sign_q
	//   S+1..4  = |a|
	//   S+5..8  = |b|

	// Two-DIVQ long-division of |a| by |b|.lo (at S+7).
	ldd	#0
	ldw	1,s			// W = |a|.hi
	divq	7,s			// → W = q_hi, D = remainder
	stw	,x			// *X = |q|.hi
	ldw	3,s			// W = |a|.lo (D still has remainder)
	divq	7,s			// → W = q_lo, D = final remainder
	stw	2,x			// *(X+2) = |q|.lo

	// Apply sign to *X if sign_q.MSB is set (negative quotient).
	tst	,s
	bpl	.Lsd32q_done
	ldd	#0
	subd	2,x
	std	2,x
	ldd	#0
	sbcd	,x
	std	,x
.Lsd32q_done:
	leas	9,s			// pop sign(1) + |a|(4) + |b|(4)
	puls	y
	rts

.Lsd32q_slow:
	// Eligibility failed. Pop our workspace + saved Y, fall through
	// to the slow path which does its own pshs/leas.
	leas	8,s
	puls	y
#endif
	pshs	y
	leas	-10,s		// quot(4) + rem(4) + counter(1) + sign(1)
	// Frame layout (relative to S):
	//   S+0..3   = quotient
	//   S+4..7   = remainder
	//   S+8      = counter
	//   S+9      = sign flag
	//   S+10..11 = saved Y
	//   S+12..13 = return addr
	//   S+14..15 = a_hi
	//   S+16..17 = a_lo
	//   S+18..19 = b_hi
	//   S+20..21 = b_lo

	// Determine sign: XOR high bytes of a_hi and b_hi
	lda	14,s		// a_hi high byte
	eora	18,s		// XOR b_hi high byte
	sta	9,s		// save sign flag

	// Copy dividend to quotient workspace
	ldd	14,s
	std	,s
	ldd	16,s
	std	2,s

	// Make a (quotient) positive if needed
	tst	,s		// test MSB of a_hi
	bpl	.Lsd32_a_pos
	// Negate quotient (32-bit): complement + 1
	// NOTE: use EOR not COM for hi word -- COM sets carry, corrupting chain
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsd32_a_pos:

	// Make b (divisor) positive if needed (in place on stack args)
	tst	18,s		// test MSB of b_hi
	bpl	.Lsd32_b_pos
	ldd	20,s		// b_lo
	coma
	comb
	addd	#1
	std	20,s
	ldd	18,s		// b_hi
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	18,s
.Lsd32_b_pos:

	// Init remainder = 0
	ldd	#0
	std	4,s
	std	6,s

	// Counter = 32
	lda	#32
	sta	8,s

.Lsd32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	18,s
	bhi	.Lsd32_sub
	blo	.Lsd32_skip
	ldd	6,s
	cmpd	20,s
	blo	.Lsd32_skip

.Lsd32_sub:
	ldd	6,s
	subd	20,s
	std	6,s
	lda	5,s
	sbca	19,s
	sta	5,s
	lda	4,s
	sbca	18,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lsd32_skip:
	dec	8,s
	bne	.Lsd32_loop

	// Negate quotient if sign flag says so
	tst	9,s
	bpl	.Lsd32_pos
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsd32_pos:

	// Copy quotient to caller's sret buffer
	ldd	,s
	std	,x
	ldd	2,s
	std	2,x

	leas	10,s
	puls	y
	rts

// ============================================================
// __modsi3 -- signed 32-bit remainder
// Remainder sign = sign of dividend (a).
// ============================================================
	.globl	__modsi3
	.type	__modsi3,@function
__modsi3:
#ifdef __6309__
	// HD6309 fast path mirroring __divsi3's. Difference: sign of
	// remainder follows sign(a) (no XOR with sign(b)), and we extract
	// the FINAL remainder from D after the second DIVQ rather than
	// the quotient.
	pshs	y
	leas	-8,s
	// Frame: same shape as __divsi3 fast path
	//   S+0..3=|a|, S+4..7=|b|, S+8..9=Y, S+10..11=ret,
	//   S+12..13=a_hi, S+14..15=a_lo, S+16..17=b_hi, S+18..19=b_lo

	// Compute |b| in workspace via branchless 0 - value chain.
	ldd	16,s
	std	4,s
	ldd	18,s
	std	6,s
	tst	4,s
	bpl	.Lsm32q_b_pos
	ldd	#0
	subd	6,s
	std	6,s
	ldd	#0
	sbcd	4,s
	std	4,s
.Lsm32q_b_pos:

	ldd	4,s			// |b|.hi
	bne	.Lsm32q_slow
	ldd	6,s			// |b|.lo
	beq	.Lsm32q_slow
	bmi	.Lsm32q_slow

	// Compute |a| in workspace
	ldd	12,s
	std	,s
	ldd	14,s
	std	2,s
	tst	,s
	bpl	.Lsm32q_a_pos
	ldd	#0
	subd	2,s
	std	2,s
	ldd	#0
	sbcd	,s
	std	,s
.Lsm32q_a_pos:

	// Save sign(a_orig). Remainder follows sign of a regardless of
	// sign(b).
	lda	12,s			// a_hi.MSB
	pshs	a
	// S+0=sign_a, S+1..4=|a|, S+5..8=|b|

	// Two-DIVQ. Final remainder is in D after the second DIVQ.
	// |b|.lo high half is 0 by eligibility; remainder fits in 16 bits.
	ldd	#0
	ldw	1,s			// |a|.hi
	divq	7,s			// W = q_hi (discard), D = remainder
	ldw	3,s			// |a|.lo
	divq	7,s			// W = q_lo (discard), D = final remainder
	// Remainder is 16-bit (≤ |b|.lo - 1 < 0x8000); the 32-bit signed
	// remainder has the same magnitude. Sign-extend (or zero-extend)
	// into the high half of *X.
	clr	,x
	clr	1,x
	std	2,x

	// Apply sign(a) to *X if a was negative.
	tst	,s
	bpl	.Lsm32q_done
	ldd	#0
	subd	2,x
	std	2,x
	ldd	#0
	sbcd	,x
	std	,x
.Lsm32q_done:
	leas	9,s			// pop sign(1) + |a|(4) + |b|(4)
	puls	y
	rts

.Lsm32q_slow:
	leas	8,s
	puls	y
#endif
	pshs	y
	leas	-10,s
	// Frame layout (relative to S):
	//   S+0..3   = quotient
	//   S+4..7   = remainder
	//   S+8      = counter
	//   S+9      = sign flag (sign of a)
	//   S+10..11 = saved Y
	//   S+12..13 = return addr
	//   S+14..15 = a_hi
	//   S+16..17 = a_lo
	//   S+18..19 = b_hi
	//   S+20..21 = b_lo

	// Save sign of a (remainder follows dividend sign)
	lda	14,s		// a_hi high byte
	sta	9,s

	// Copy dividend to quotient workspace
	ldd	14,s
	std	,s
	ldd	16,s
	std	2,s

	// Make a positive if needed
	tst	,s
	bpl	.Lsm32_a_pos
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsm32_a_pos:

	// Make b positive if needed
	tst	18,s
	bpl	.Lsm32_b_pos
	ldd	20,s
	coma
	comb
	addd	#1
	std	20,s
	ldd	18,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	18,s
.Lsm32_b_pos:

	ldd	#0
	std	4,s
	std	6,s

	lda	#32
	sta	8,s

.Lsm32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	18,s
	bhi	.Lsm32_sub
	blo	.Lsm32_skip
	ldd	6,s
	cmpd	20,s
	blo	.Lsm32_skip

.Lsm32_sub:
	ldd	6,s
	subd	20,s
	std	6,s
	lda	5,s
	sbca	19,s
	sta	5,s
	lda	4,s
	sbca	18,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lsm32_skip:
	dec	8,s
	bne	.Lsm32_loop

	// Negate remainder if a was negative
	tst	9,s
	bpl	.Lsm32_pos
	ldd	6,s
	coma
	comb
	addd	#1
	std	6,s
	ldd	4,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	4,s
.Lsm32_pos:

	// Copy remainder to caller's sret buffer
	ldd	4,s
	std	,x
	ldd	6,s
	std	2,x

	leas	10,s
	puls	y
	rts
