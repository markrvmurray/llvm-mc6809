//===-- umulodi4.S - 64-bit unsigned multiply with overflow ----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// di_uint __umulodi4(di_uint *result, di_uint a, di_uint b, int *overflow);
//
// 64x64 -> 128 multiply, returning the low 64 bits in *result and
// setting *overflow non-zero iff the high 64 bits are non-zero.
//
// Bug #239: replaces the inline 4-op decomposition that
// `MC6809LegalizerInfo::legalizeMultiplyWithOverflow` was emitting
// for G_UMULO i64 (G_MUL + G_UDIV + G_UMAX + G_ICMP, all i64), which
// blew the 5-slot ACC32 register class on HD6309 -O2/-Og and wiped
// out the O2-hd6309-mame and Og-hd6309-mame bench levels (130/130
// BUILDFAIL each) by failing to allocate strtoull / strtoumax.
// Hiding the i64 vregs inside this library frame relieves the
// caller's regalloc pressure entirely.
//
// gcc6809 ABI v1 (sret + 4-arg, mirrors __muldi3 + __udivmoddi4):
//   IX            = sret pointer (caller-allocated 8-byte buffer for low 64)
//   2,S..9,S      = a (8 bytes, big-endian: 2,S = MSB, 9,S = LSB)
//   10,S..17,S    = b (8 bytes, big-endian: 10,S = MSB, 17,S = LSB)
//   18,S..19,S    = overflow_ptr (caller-allocated int* for overflow flag)
// On exit:
//   IX[0..7]      = low 64 bits of a*b (MSB at IX, LSB at IX+7)
//   *overflow_ptr = 1 if high 64 bits non-zero, else 0
// Clobbers D. Preserves Y, U.
//
// Algorithm: schoolbook 64x64 -> 128 multiplication. Allocates a
// 16-byte virtual product on the stack at 0,S..15,S (S+0 = MSB =
// bits 120-127, S+15 = LSB = bits 0-7). For each pair (i, j) of
// byte indices in [0..7], compute the 8x8 -> 16 partial product
// ai * bj and accumulate into the virtual buffer at byte positions
// (i+j) for the high half and (i+j+1) for the low half, with carry
// propagating upward (toward MSB, toward S+0).
//
// Partials are emitted in significance-descending order (i+j=0
// first, ..., i+j=14 last), so that when a later partial generates
// a carry up, it propagates through bytes that have already been
// computed by earlier partials. Carries OUT of S+0 are dropped
// (they belong to bits 128+, outside the 128-bit truncated result).
//
// After all 64 partials:
//   - Copy bytes 8..15 of the virtual buffer to IX[0..7] (low 64).
//   - OR bytes 0..7 together; non-zero => overflow.
//   - Store overflow flag at *overflow_ptr.
//
// HD6309 fast path NOT yet wired (Bug #239 follow-up). The
// schoolbook MC6809 path produces correct results on both
// processors; HD6309 just runs more cycles than necessary. If a
// hot test regresses materially in the bench, add an
// `#ifdef __6309__` MULD-based fast path mirroring Bug #233's
// __muldi3 HD6309 section, extended to 16 partials.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl __umulodi4
	.type  __umulodi4,@function
__umulodi4:
	// Allocate 16-byte virtual product on stack.
	// Frame after leas -16,s:
	//   0,S..15,S   = virtual product (S+0 = MSB, S+15 = LSB)
	//   16,S..17,S  = saved return address
	//   18,S..25,S  = a (was 2,S..9,S at entry)
	//   26,S..33,S  = b (was 10,S..17,S at entry)
	//   34,S..35,S  = overflow_ptr (was 18,S..19,S at entry)
	leas	-16,s

	// Initialize 16-byte virtual product to zero.
	clr	,s
	clr	1,s
	clr	2,s
	clr	3,s
	clr	4,s
	clr	5,s
	clr	6,s
	clr	7,s
	clr	8,s
	clr	9,s
	clr	10,s
	clr	11,s
	clr	12,s
	clr	13,s
	clr	14,s
	clr	15,s

	// ============================================================
	// i+j = 0  (high byte to S+0, low byte to S+1)
	// ============================================================
	// 1 partial: (0,0)

	lda	18,s		// a[0] (MSB)
	ldb	26,s		// b[0] (MSB)
	mul
	addb	1,s		// low byte to S+1
	stb	1,s
	adca	,s		// high byte to S+0
	sta	,s

	// ============================================================
	// i+j = 1  (high byte to S+1, low byte to S+2)
	// ============================================================
	// 2 partials: (0,1) (1,0)

	lda	18,s
	ldb	27,s
	mul
	addb	2,s
	stb	2,s
	adca	1,s
	sta	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	26,s
	mul
	addb	2,s
	stb	2,s
	adca	1,s
	sta	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 2  (high byte to S+2, low byte to S+3)
	// ============================================================
	// 3 partials: (0,2) (1,1) (2,0)

	lda	18,s
	ldb	28,s
	mul
	addb	3,s
	stb	3,s
	adca	2,s
	sta	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	27,s
	mul
	addb	3,s
	stb	3,s
	adca	2,s
	sta	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	26,s
	mul
	addb	3,s
	stb	3,s
	adca	2,s
	sta	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 3  (high byte to S+3, low byte to S+4)
	// ============================================================
	// 4 partials: (0,3) (1,2) (2,1) (3,0)

	lda	18,s
	ldb	29,s
	mul
	addb	4,s
	stb	4,s
	adca	3,s
	sta	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	28,s
	mul
	addb	4,s
	stb	4,s
	adca	3,s
	sta	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	27,s
	mul
	addb	4,s
	stb	4,s
	adca	3,s
	sta	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	26,s
	mul
	addb	4,s
	stb	4,s
	adca	3,s
	sta	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 4  (high byte to S+4, low byte to S+5)
	// ============================================================
	// 5 partials: (0,4) (1,3) (2,2) (3,1) (4,0)

	lda	18,s
	ldb	30,s
	mul
	addb	5,s
	stb	5,s
	adca	4,s
	sta	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	29,s
	mul
	addb	5,s
	stb	5,s
	adca	4,s
	sta	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	28,s
	mul
	addb	5,s
	stb	5,s
	adca	4,s
	sta	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	27,s
	mul
	addb	5,s
	stb	5,s
	adca	4,s
	sta	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	26,s
	mul
	addb	5,s
	stb	5,s
	adca	4,s
	sta	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 5  (high byte to S+5, low byte to S+6)
	// ============================================================
	// 6 partials: (0,5) (1,4) (2,3) (3,2) (4,1) (5,0)

	lda	18,s
	ldb	31,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	30,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	29,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	28,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	27,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	26,s
	mul
	addb	6,s
	stb	6,s
	adca	5,s
	sta	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 6  (high byte to S+6, low byte to S+7)
	// ============================================================
	// 7 partials: (0,6) (1,5) (2,4) (3,3) (4,2) (5,1) (6,0)

	lda	18,s
	ldb	32,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	31,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	30,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	29,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	28,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	27,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	26,s
	mul
	addb	7,s
	stb	7,s
	adca	6,s
	sta	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 7  (high byte to S+7, low byte to S+8 — straddles)
	// ============================================================
	// 8 partials: (0,7) (1,6) (2,5) (3,4) (4,3) (5,2) (6,1) (7,0)

	lda	18,s
	ldb	33,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	19,s
	ldb	32,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	31,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	30,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	29,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	28,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	27,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	26,s
	mul
	addb	8,s
	stb	8,s
	adca	7,s
	sta	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 8  (high byte to S+8, low byte to S+9)
	// ============================================================
	// 7 partials: (1,7) (2,6) (3,5) (4,4) (5,3) (6,2) (7,1)

	lda	19,s
	ldb	33,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	20,s
	ldb	32,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	31,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	30,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	29,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	28,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	27,s
	mul
	addb	9,s
	stb	9,s
	adca	8,s
	sta	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 9  (high byte to S+9, low byte to S+10)
	// ============================================================
	// 6 partials: (2,7) (3,6) (4,5) (5,4) (6,3) (7,2)

	lda	20,s
	ldb	33,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	21,s
	ldb	32,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	31,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	30,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	29,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	28,s
	mul
	addb	10,s
	stb	10,s
	adca	9,s
	sta	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 10  (high byte to S+10, low byte to S+11)
	// ============================================================
	// 5 partials: (3,7) (4,6) (5,5) (6,4) (7,3)

	lda	21,s
	ldb	33,s
	mul
	addb	11,s
	stb	11,s
	adca	10,s
	sta	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	22,s
	ldb	32,s
	mul
	addb	11,s
	stb	11,s
	adca	10,s
	sta	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	31,s
	mul
	addb	11,s
	stb	11,s
	adca	10,s
	sta	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	30,s
	mul
	addb	11,s
	stb	11,s
	adca	10,s
	sta	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	29,s
	mul
	addb	11,s
	stb	11,s
	adca	10,s
	sta	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 11  (high byte to S+11, low byte to S+12)
	// ============================================================
	// 4 partials: (4,7) (5,6) (6,5) (7,4)

	lda	22,s
	ldb	33,s
	mul
	addb	12,s
	stb	12,s
	adca	11,s
	sta	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	23,s
	ldb	32,s
	mul
	addb	12,s
	stb	12,s
	adca	11,s
	sta	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	31,s
	mul
	addb	12,s
	stb	12,s
	adca	11,s
	sta	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	30,s
	mul
	addb	12,s
	stb	12,s
	adca	11,s
	sta	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 12  (high byte to S+12, low byte to S+13)
	// ============================================================
	// 3 partials: (5,7) (6,6) (7,5)

	lda	23,s
	ldb	33,s
	mul
	addb	13,s
	stb	13,s
	adca	12,s
	sta	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	24,s
	ldb	32,s
	mul
	addb	13,s
	stb	13,s
	adca	12,s
	sta	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	31,s
	mul
	addb	13,s
	stb	13,s
	adca	12,s
	sta	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 13  (high byte to S+13, low byte to S+14)
	// ============================================================
	// 2 partials: (6,7) (7,6)

	lda	24,s
	ldb	33,s
	mul
	addb	14,s
	stb	14,s
	adca	13,s
	sta	13,s
	ldb	12,s
	adcb	#0
	stb	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	lda	25,s
	ldb	32,s
	mul
	addb	14,s
	stb	14,s
	adca	13,s
	sta	13,s
	ldb	12,s
	adcb	#0
	stb	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// i+j = 14  (high byte to S+14, low byte to S+15 — LSB pair)
	// ============================================================
	// 1 partial: (7,7)

	lda	25,s
	ldb	33,s
	mul
	addb	15,s
	stb	15,s
	adca	14,s
	sta	14,s
	ldb	13,s
	adcb	#0
	stb	13,s
	ldb	12,s
	adcb	#0
	stb	12,s
	ldb	11,s
	adcb	#0
	stb	11,s
	ldb	10,s
	adcb	#0
	stb	10,s
	ldb	9,s
	adcb	#0
	stb	9,s
	ldb	8,s
	adcb	#0
	stb	8,s
	ldb	7,s
	adcb	#0
	stb	7,s
	ldb	6,s
	adcb	#0
	stb	6,s
	ldb	5,s
	adcb	#0
	stb	5,s
	ldb	4,s
	adcb	#0
	stb	4,s
	ldb	3,s
	adcb	#0
	stb	3,s
	ldb	2,s
	adcb	#0
	stb	2,s
	ldb	1,s
	adcb	#0
	stb	1,s
	ldb	,s
	adcb	#0
	stb	,s

	// ============================================================
	// Copy LOW 64 bits (S+8..S+15) to caller's sret buffer at IX[0..7].
	// ============================================================

	ldd	8,s
	std	,x
	ldd	10,s
	std	2,x
	ldd	12,s
	std	4,x
	ldd	14,s
	std	6,x

	// ============================================================
	// Compute *overflow_ptr = (HIGH 64 != 0) ? 1 : 0.
	// HIGH 64 lives at S+0..S+7. OR all eight bytes; non-zero
	// in A => overflow.
	// ============================================================

	lda	,s
	ora	1,s
	ora	2,s
	ora	3,s
	ora	4,s
	ora	5,s
	ora	6,s
	ora	7,s
	beq	.Lno_overflow
	ldd	#1		// 16-bit: high=0, low=1
	bra	.Lof_store
.Lno_overflow:
	ldd	#0		// 16-bit zero
.Lof_store:
	// `int` on MC6809 is 2 bytes big-endian: STD writes high byte at
	// *X and low byte at *(X+1).  Store through X, not Y: Y is
	// callee-saved (MC6809 CSR = IY,SU) and this routine promises to
	// preserve it (see header).  X held the sret pointer but is dead
	// after the i64 result copy above and is caller-clobberable, so it
	// is the correct scratch here.  Bug #351: the previous `ldy 34,s`
	// silently clobbered the caller's Y; when the caller kept a live
	// value in Y across this call (e.g. strtoull's loop pointer under
	// greedy regalloc) the value was destroyed, yielding
	// strtoull("ff",16)=0x0f instead of 0xff.
	ldx	34,s
	std	,x

	leas	16,s
	rts
	.size __umulodi4, . - __umulodi4
