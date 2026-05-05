//===-- divhi.S - 16-bit division for MC6809 ---------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// 16-bit signed and unsigned division and remainder.
//
// All routines: X = first arg, 2,S = second arg (after return addr).
// Returns: X = result. Clobbers: D. Preserves: Y, U.
//
// Algorithm: restoring division (shift-and-subtract), 16 iterations.
//
// HD6309 (`__6309__`): the unsigned variants take a fast path when the
// divisor fits in the signed-DIVD-safe range. Long-division by an
// 8-bit divisor uses two DIVD instructions: ~50 cycles vs ~400 for the
// 16-iteration loop, ~8x speedup. Bug #161 Phase 3.
//
// DIVD semantics on HD6309: D ÷ src8 (both signed) → B = quotient,
// A = remainder; V flag set on overflow. The fast path constrains
// divisor ∈ [2, 127] so:
//   - the first DIVD with D = (0:dividend.hi) sees a non-negative
//     dividend (high bit of A clear) and the quotient never exceeds
//     127 (since 255/2 = 127, fits in i8 signed);
//   - the remainder ≤ divisor-1 ≤ 126 (high bit clear), so the
//     second DIVD with D = (remainder:dividend.lo) is also signed-safe.
// divisor=1 is handled separately (return X unchanged; quotient = a).
// divisor=0 falls through to the loop (undefined behavior; matches
// existing semantics).
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

// ============================================================
// __udivhi3 -- unsigned 16-bit division (returns quotient)
// ============================================================
	.globl	__udivhi3
	.type	__udivhi3,@function
__udivhi3:
#ifdef __6309__
	// HD6309 fast path test — runs before any stack mutation, so
	// divisor (the 2nd arg) sits at S+2..3 and dividend is in X.
	ldd	2,s		// D = divisor (A=hi, B=lo)
	tsta			// divisor.hi == 0 ?
	bne	.Lud_slow	// no → loop path
	tstb
	beq	.Lud_slow	// divisor == 0 → loop (undefined-behavior match)
	bmi	.Lud_slow	// divisor.lo ≥ 128 → loop (signed-DIVD safety;
				//   must test before CMPB which would clobber N)
	cmpb	#1
	beq	.Lud_div_by_1	// divisor == 1 → quotient = dividend
	// divisor ∈ [2, 127], in B. X = dividend.
	pshs	y		// save callee-saved Y
	pshs	b		// push divisor8 at S+0
	// S: 0=divisor8, 1..2=Y, 3..4=ret, 5..6=divisor16
	tfr	x,d		// D = dividend (A=hi, B=lo)
	pshs	b		// save dividend.lo at S+0
	// S: 0=dividend.lo, 1=divisor8, 2..3=Y, 4..5=ret, 6..7=divisor16
	tfr	a,b		// B = dividend.hi
	clra			// A = 0 → D = (0 : dividend.hi), unsigned 0..255
	divd	1,s		// D / divisor8 → B=q_hi, A=remainder ∈ [0,126]
	pshs	b		// save q_hi at S+0
	// S: 0=q_hi, 1=dividend.lo, 2=divisor8, 3..4=Y, 5..6=ret, 7..8=divisor16
	ldb	1,s		// B = dividend.lo (A still has remainder, hi-bit clear)
	divd	2,s		// D / divisor8 → B=q_lo, A=final remainder
	lda	,s		// A = q_hi (overwriting remainder)
	tfr	d,x		// X = quotient (A:B = q_hi:q_lo)
	leas	3,s		// pop q_hi, dividend.lo, divisor8
	puls	y
	rts

.Lud_div_by_1:
	rts			// X already = dividend = quotient

.Lud_slow:
#endif
	pshs	y		// save callee-saved Y
	// stack: [Y:2][ret:2][divisor:2]  offsets 0,2,4
	tfr	x,d		// D = dividend (becomes quotient)
	ldy	#0		// Y = remainder
	ldx	#16		// loop counter
.Lud_loop:
	aslb			// shift quotient left
	rola
	exg	d,y		// D = remainder, Y = partial quotient
	rolb			// shift carry (from dividend MSB) into remainder
	rola
	cmpd	4,s		// remainder >= divisor?
	blo	.Lud_skip
	subd	4,s		// remainder -= divisor
	exg	d,y		// swap back: D = quotient, Y = remainder
	orb	#1		// set quotient bit
	bra	.Lud_next
.Lud_skip:
	exg	d,y		// swap back (quotient bit stays 0)
.Lud_next:
	leax	-1,x
	bne	.Lud_loop
	tfr	d,x		// X = quotient
	puls	y
	rts

// ============================================================
// __umodhi3 -- unsigned 16-bit remainder
// ============================================================
	.globl	__umodhi3
	.type	__umodhi3,@function
__umodhi3:
#ifdef __6309__
	// Same divisor-range test as __udivhi3, but return remainder.
	ldd	2,s		// D = divisor
	tsta
	bne	.Lum_slow
	tstb
	beq	.Lum_slow	// divisor == 0 → loop
	bmi	.Lum_slow	// divisor.lo ≥ 128 → loop (test before CMPB)
	cmpb	#1
	beq	.Lum_div_by_1	// divisor == 1 → remainder = 0
	pshs	y
	pshs	b		// divisor8 at S+0
	// S: 0=divisor8, 1..2=Y, 3..4=ret, 5..6=divisor16
	tfr	x,d
	pshs	b		// dividend.lo at S+0
	// S: 0=dividend.lo, 1=divisor8, 2..3=Y, 4..5=ret, 6..7=divisor16
	tfr	a,b
	clra
	divd	1,s		// (0:hi)/divisor → B=q_hi, A=remainder
	ldb	,s		// B = dividend.lo (A still has remainder, hi-bit clear)
	divd	1,s		// (rem:lo)/divisor → B=q_lo, A=final remainder
	tfr	a,b		// B = remainder
	clra			// A = 0 → D = (0 : remainder)
	tfr	d,x		// X = remainder
	leas	2,s		// pop dividend.lo, divisor8
	puls	y
	rts

.Lum_div_by_1:
	ldx	#0		// remainder = 0
	rts

.Lum_slow:
#endif
	pshs	y
	tfr	x,d
	ldy	#0
	ldx	#16
.Lum_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	4,s
	blo	.Lum_skip
	subd	4,s
	exg	d,y
	orb	#1
	bra	.Lum_next
.Lum_skip:
	exg	d,y
.Lum_next:
	leax	-1,x
	bne	.Lum_loop
	tfr	y,x		// X = remainder (not quotient)
	puls	y
	rts

// ============================================================
// __divhi3 -- signed 16-bit division (returns quotient)
// Result sign = sign(a) XOR sign(b).
// ============================================================
	.globl	__divhi3
	.type	__divhi3,@function
__divhi3:
#ifdef __6309__
	// HD6309 fast path: if |b| ∈ [2, 127] use two-DIVD long-division
	// (same constraint as __udivhi3; DIVD is signed so we do abs(a)/abs(b)
	// then fix the quotient sign).
	// Pre-pshs frame: X=a, S+0..1=ret, S+2..3=b.
	tfr	x,d		// D = a: need a.hi to compute sign_q below.
	eora	2,s		// A.bit7 = sign(a) XOR sign(b) = sign_q (other bits irrelevant).
	ldd	2,s		// D = b; N = sign(b).
	bpl	.Lsd6_pos	// b ≥ 0
	negd			// D = |b|  (HD6309 NEGD; V set on -32768)
	bvs	.Lsd6_slow	// b == -32768 → |b| overflows to 0x8000 → slow
	bra	.Lsd6_check
.Lsd6_pos:
	tstd
	beq	.Lsd6_slow	// b == 0 → undefined, slow
.Lsd6_check:
	// D = |b|. Verify A (=|b|.hi) == 0 and B (=|b|.lo) ∈ [2, 127].
	tsta
	bne	.Lsd6_slow	// |b| ≥ 256
	cmpb	#128
	bhs	.Lsd6_slow	// |b| ≥ 128
	cmpb	#1
	beq	.Lsd6_one	// |b| == 1 → quotient = ±a
	// B = |b| ∈ [2, 127]. Hold sign_q in Y (not on stack) to avoid a
	// MAME CC-flag interaction with the `tst ,s` / `leas` sequence.
	// Y is callee-saved; we save old Y, use it as a sign flag, restore.
	pshs	y		// save callee-saved Y. S: 0..1=Y, 2..3=ret, 4..5=b.
	pshs	b		// push |b|. S: 0=|b|, 1..2=Y, 3..4=ret, 5..6=b.
	// Compute sign_q = sign(a) XOR sign(b) → Y (0xFFFF=negative, 0x0000=positive).
	// SEX extends B's sign into A only; B unchanged. Two-step to get 0x0000/0xFFFF:
	//   step1: tfr a,b; sex → A=sign_ext(B.bit7), B=original_B
	//   step2: tfr a,b; sex → A=sign_ext(A.bit7) = same, B=0x00 or 0xFF → D=0x0000 or 0xFFFF
	tfr	x,d		// D = a; A = a.hi.
	eora	5,s		// A.bit7 = sign(a) XOR sign(b); b_orig at S+5.
	tfr	a,b		// B = sign_q byte.
	sex			// A = 0x00 or 0xFF (extended from B.bit7); B = original sign_q byte.
	tfr	a,b		// B = extended sign (0x00 or 0xFF).
	sex			// D = (0x0000) or (0xFFFF).
	tfr	d,y		// Y = sign_q indicator: 0x0000 or 0xFFFF.
	// Compute |a|. TFR does NOT update CC; TSTA sets N = sign(a).
	tfr	x,d		// D = a.
	tsta			// N = a.hi.bit7 = sign(a). (Needed: double-sex above left N from sign_q.)
	bpl	.Lsd6_a_pos
	negd			// D = |a|
.Lsd6_a_pos:
	// D = |a|. Two-DIVD long-division with |b| at S+1.
	pshs	b		// push |a|.lo. S: 0=|a|.lo, 1=|b|, 2..3=Y, 4..5=ret, 6..7=b.
	tfr	a,b		// B = |a|.hi
	clra
	divd	1,s		// (0:|a|.hi) / |b| → B=q_hi, A=rem; |b| at S+1.
	pshs	b		// push q_hi. S: 0=q_hi, 1=|a|.lo, 2=|b|, 3..4=Y, 5..6=ret, 7..8=b.
	ldb	1,s		// B = |a|.lo; A = rem from Phase 1.
	divd	2,s		// (rem:|a|.lo) / |b| → B=q_lo, A=final_rem; |b| at S+2.
	lda	,s		// A = q_hi.
	// D = |quotient|. Apply sign from Y.
	leas	2,s		// pop q_hi, |a|.lo. S: 0=|b|, 1..2=Y, 3..4=ret, 5..6=b.
	cmpy	#0		// sign_q == 0 → positive quotient
	beq	.Lsd6_q_done
	negd			// D = signed quotient.
.Lsd6_q_done:
	tfr	d,x		// X = quotient.
	leas	1,s		// pop |b|. S: 0..1=Y, 2..3=ret, 4..5=b.
	puls	y
	rts

.Lsd6_one:
	// |b| == 1: quotient = a if b=1, or -a if b=-1.
	// No stack mutation in this path.
	ldd	2,s		// reload original b.
	bpl	.Lsd6_one_rts	// b=1 → quotient = a = X.
	tfr	x,d		// b=-1 → quotient = -a.
	negd
	tfr	d,x
.Lsd6_one_rts:
	rts

.Lsd6_slow:
#endif
	pshs	y
	// [Y:2][ret:2][b:2]  offsets 0,2,4
	// Determine result sign: XOR high bytes of a and b
	tfr	x,d		// D = a
	eora	4,s		// A.7 = sign(a) XOR sign(b)
	pshs	a		// save sign flag
	// [sign:1][Y:2][ret:2][b:2]  offsets 0,1,3,5
	// Make a positive
	tfr	x,d		// D = a (tfr does NOT set flags)
	tsta			// test high byte for sign
	bpl	.Lsd_a_pos
	coma			// negate D (two's complement)
	comb
	addd	#1
.Lsd_a_pos:
	pshs	d		// save |a|
	// [|a|:2][sign:1][Y:2][ret:2][b:2]  offsets 0,2,3,5,7
	// Make b positive
	ldd	7,s		// load b
	bpl	.Lsd_b_pos
	coma
	comb
	addd	#1
	std	7,s		// store |b| back
.Lsd_b_pos:
	// Unsigned division: |a| / |b|
	puls	d		// D = |a|
	// [sign:1][Y:2][ret:2][|b|:2]  offsets 0,1,3,5
	ldy	#0
	ldx	#16
.Lsd_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	5,s		// compare with |b|
	blo	.Lsd_skip
	subd	5,s
	exg	d,y
	orb	#1
	bra	.Lsd_next
.Lsd_skip:
	exg	d,y
.Lsd_next:
	leax	-1,x
	bne	.Lsd_loop
	// D = quotient -- fix sign if needed
	tst	,s		// check sign flag (bit 7)
	bpl	.Lsd_pos
	coma			// negate quotient
	comb
	addd	#1
.Lsd_pos:
	tfr	d,x		// X = signed quotient
	leas	1,s		// drop sign byte
	puls	y
	rts

// ============================================================
// __modhi3 -- signed 16-bit remainder
// Remainder sign = sign of dividend (a).
// ============================================================
	.globl	__modhi3
	.type	__modhi3,@function
__modhi3:
#ifdef __6309__
	// Same fast path as __divhi3 but return remainder.
	// sign_r = sign(a); no need to XOR with sign(b).
	// Pre-pshs frame: X=a, S+0..1=ret, S+2..3=b.
	ldd	2,s		// D = b; N = sign(b).
	bpl	.Lsm6_pos
	negd			// D = |b|
	bvs	.Lsm6_slow
	bra	.Lsm6_check
.Lsm6_pos:
	tstd
	beq	.Lsm6_slow
.Lsm6_check:
	tsta
	bne	.Lsm6_slow
	cmpb	#128
	bhs	.Lsm6_slow
	cmpb	#1
	beq	.Lsm6_one	// |b| == 1 → remainder = 0
	// B = |b| ∈ [2, 127]. Hold sign_r in Y to avoid tst ,s / leas MAME bug.
	pshs	y		// save old Y. S: 0..1=Y, 2..3=ret, 4..5=b.
	pshs	b		// push |b|. S: 0=|b|, 1..2=Y, 3..4=ret, 5..6=b.
	// sign_r = sign(a) → Y (0xFFFF or 0x0000). Same two-step SEX.
	tfr	x,d		// D = a; A = a.hi; N = sign(a).
	tfr	a,b		// B = a.hi.
	sex			// A = 0x00 or 0xFF.
	tfr	a,b		// B = extended sign.
	sex			// D = 0x0000 or 0xFFFF.
	tfr	d,y		// Y = sign_r indicator.
	// Compute |a|. Use TSTA to set N = sign(a) (double-sex may have left N wrong).
	tfr	x,d		// D = a.
	tsta			// N = a.hi.bit7 = sign(a).
	bpl	.Lsm6_a_pos
	negd			// D = |a|
.Lsm6_a_pos:
	// D = |a|. Two-DIVD long-division with |b| at S+1.
	pshs	b		// push |a|.lo. S: 0=|a|.lo, 1=|b|, 2..3=Y, 4..5=ret, 6..7=b.
	tfr	a,b
	clra
	divd	1,s		// (0:|a|.hi) / |b| → B=q_hi, A=rem; |b| at S+1.
	ldb	,s		// B = |a|.lo; A = rem.
	divd	1,s		// (rem:|a|.lo) / |b| → B=q_lo, A=|remainder|; |b| at S+1.
	// A = |remainder|. Apply sign from Y.
	tfr	a,b		// B = |remainder|.
	leas	1,s		// pop |a|.lo. S: 0=|b|, 1..2=Y, 3..4=ret, 5..6=b.
	cmpy	#0		// sign_r == 0 → positive remainder
	beq	.Lsm6_r_done
	negb			// B = -|remainder|
.Lsm6_r_done:
	sex			// sign-extend B into D.
	tfr	d,x		// X = i16 remainder.
	leas	1,s		// pop |b|. S: 0..1=Y, 2..3=ret, 4..5=b.
	puls	y
	rts

.Lsm6_one:
	ldx	#0		// |b|=1 → remainder = 0.
	rts

.Lsm6_slow:
#endif
	pshs	y
	// [Y:2][ret:2][b:2]
	// Save sign of a (remainder follows dividend sign)
	tfr	x,d
	pshs	a		// save sign of a
	// [asign:1][Y:2][ret:2][b:2]  offsets 0,1,3,5
	// Make a positive
	tfr	x,d		// D = a (tfr does NOT set flags)
	tsta			// test high byte for sign
	bpl	.Lsm_a_pos
	coma
	comb
	addd	#1
.Lsm_a_pos:
	pshs	d		// save |a|
	// [|a|:2][asign:1][Y:2][ret:2][b:2]  offsets 0,2,3,5,7
	// Make b positive
	ldd	7,s
	bpl	.Lsm_b_pos
	coma
	comb
	addd	#1
	std	7,s
.Lsm_b_pos:
	// Unsigned division to get remainder
	puls	d		// D = |a|
	// [asign:1][Y:2][ret:2][|b|:2]  offsets 0,1,3,5
	ldy	#0
	ldx	#16
.Lsm_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	5,s
	blo	.Lsm_skip
	subd	5,s
	exg	d,y
	orb	#1
	bra	.Lsm_next
.Lsm_skip:
	exg	d,y
.Lsm_next:
	leax	-1,x
	bne	.Lsm_loop
	// Y = remainder -- fix sign
	tfr	y,d
	tst	,s		// check sign of original a
	bpl	.Lsm_pos
	coma
	comb
	addd	#1
.Lsm_pos:
	tfr	d,x		// X = signed remainder
	leas	1,s		// drop sign byte
	puls	y
	rts