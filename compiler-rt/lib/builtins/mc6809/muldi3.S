//===-- muldi3.S - 64-bit truncated multiply for MC6809 -------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// di_int __muldi3(di_int a, di_int b)
//
// 64x64->64 multiply, truncated to the low 64 bits. Schoolbook long
// multiplication using thirty-six 8-bit MUL instructions.
//
// Replaces the compiler-rt C version that returned wrong answers
// for several inputs (bug #173). The C version's 32x32->64
// __muldsi3 helper has a partial-product alignment bug for the
// big-endian + int=i16 + long=i32 + long long=i64 combination
// MC6809 has. Hand-asm avoids the trap entirely.
//
// gcc6809 ABI v1 (sret-style; same as __mulsi3.S, just one byte wider):
//   IX           = sret pointer (caller-allocated 8-byte buffer)
//   2,S..9,S     = a (8 bytes, big-endian: 2,S = MSB, 9,S = LSB)
//   10,S..17,S   = b (8 bytes, big-endian: 10,S = MSB, 17,S = LSB)
// Result (MSB at IX, LSB at IX+7) written through IX. Clobbers D.
// Preserves: Y, U.
//
// Algorithm: for each pair (i, j) of byte indices in [0..7] with
// significance(ai*bj) <= 7 (i.e., i+j >= 7), accumulate the 8x8->16
// partial product into the result buffer at position (i+j-8, i+j-7),
// propagating any carry upward through higher-significance bytes
// (lower IX offsets) toward IX+0. Carries OUT of IX+0 are dropped
// (they belong to bits 64+, outside the truncated result).
//
// Partials are emitted in significance-descending order
// (i+j=7 first, ..., i+j=14 last). With this order, when a later
// partial generates a carry up, it propagates through bytes that
// have already been computed by earlier partials.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl __muldi3
	.type  __muldi3,@function
__muldi3:
	// Initialize result buffer to zero (8 bytes at IX..IX+7).
	clr	,x
	clr	1,x
	clr	2,x
	clr	3,x
	clr	4,x
	clr	5,x
	clr	6,x
	clr	7,x

	// ============================================================
	// i+j = 7  (low byte to IX+0; high byte off-end, dropped)
	// ============================================================
	// 8 partials: (0,7) (1,6) (2,5) (3,4) (4,3) (5,2) (6,1) (7,0)

	lda	2,s		// a[0] (MSB)
	ldb	17,s		// b[7] (LSB)
	mul
	addb	,x
	stb	,x

	lda	3,s		// a[1]
	ldb	16,s		// b[6]
	mul
	addb	,x
	stb	,x

	lda	4,s		// a[2]
	ldb	15,s		// b[5]
	mul
	addb	,x
	stb	,x

	lda	5,s		// a[3]
	ldb	14,s		// b[4]
	mul
	addb	,x
	stb	,x

	lda	6,s		// a[4]
	ldb	13,s		// b[3]
	mul
	addb	,x
	stb	,x

	lda	7,s		// a[5]
	ldb	12,s		// b[2]
	mul
	addb	,x
	stb	,x

	lda	8,s		// a[6]
	ldb	11,s		// b[1]
	mul
	addb	,x
	stb	,x

	lda	9,s		// a[7] (LSB)
	ldb	10,s		// b[0] (MSB)
	mul
	addb	,x
	stb	,x

	// ============================================================
	// i+j = 8  (low byte to IX+1, high byte to IX+0)
	// ============================================================
	// 7 partials: (1,7) (2,6) (3,5) (4,4) (5,3) (6,2) (7,1)

	lda	3,s		// a[1]
	ldb	17,s		// b[7]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	4,s		// a[2]
	ldb	16,s		// b[6]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	5,s		// a[3]
	ldb	15,s		// b[5]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	6,s		// a[4]
	ldb	14,s		// b[4]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	7,s		// a[5]
	ldb	13,s		// b[3]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	8,s		// a[6]
	ldb	12,s		// b[2]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	9,s		// a[7]
	ldb	11,s		// b[1]
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	// ============================================================
	// i+j = 9  (low byte to IX+2, high byte to IX+1, chain to IX+0)
	// ============================================================
	// 6 partials: (2,7) (3,6) (4,5) (5,4) (6,3) (7,2)

	lda	4,s
	ldb	17,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	5,s
	ldb	16,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	6,s
	ldb	15,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	14,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	13,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	12,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	// ============================================================
	// i+j = 10  (low to IX+3, high to IX+2, chain to IX+1, IX+0)
	// ============================================================
	// 5 partials: (3,7) (4,6) (5,5) (6,4) (7,3)

	lda	5,s
	ldb	17,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	6,s
	ldb	16,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	15,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	14,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	13,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	// ============================================================
	// i+j = 11  (low to IX+4, high to IX+3, chain to IX+2, +1, +0)
	// ============================================================
	// 4 partials: (4,7) (5,6) (6,5) (7,4)

	lda	6,s
	ldb	17,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	16,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	15,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	14,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	// ============================================================
	// i+j = 12  (low to IX+5, high to IX+4, chain to IX+3..+0)
	// ============================================================
	// 3 partials: (5,7) (6,6) (7,5)

	lda	7,s
	ldb	17,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	16,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	15,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	// ============================================================
	// i+j = 13  (low to IX+6, high to IX+5, chain to IX+4..+0)
	// ============================================================
	// 2 partials: (6,7) (7,6)

	lda	8,s
	ldb	17,s
	mul
	addb	6,x
	stb	6,x
	adca	5,x
	sta	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	16,s
	mul
	addb	6,x
	stb	6,x
	adca	5,x
	sta	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	// ============================================================
	// i+j = 14  (low to IX+7, high to IX+6, chain to IX+5..+0)
	// ============================================================
	// 1 partial: (7,7)

	lda	9,s		// a[7] (LSB)
	ldb	17,s		// b[7] (LSB)
	mul
	addb	7,x
	stb	7,x
	adca	6,x
	sta	6,x
	ldb	5,x
	adcb	#0
	stb	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	rts
	.size __muldi3, . - __muldi3
