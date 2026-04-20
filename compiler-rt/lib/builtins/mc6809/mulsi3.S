//===-- mulsi3.S - 32-bit multiply for MC6809 --------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint32_t __mulsi3(uint32_t a, uint32_t b)
//
// 32-bit unsigned multiply, truncated to 32 bits. Schoolbook long
// multiplication using ten 8-bit MUL instructions.
//
// gcc6809 ABI v1 (sret-style):
//   IX        = sret pointer (caller-allocated 4-byte buffer)
//   2,S..3,S  = a_hi
//   4,S..5,S  = a_lo
//   6,S..7,S  = b_hi
//   8,S..9,S  = b_lo
// Result (hi at IX+0, lo at IX+2; big-endian) is written through IX.
// Clobbers: D. Preserves: Y, U.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__mulsi3
	.type	__mulsi3,@function
__mulsi3:
	// ==== Step 1: a_lo * b_lo -> 32-bit result at [IX..IX+3] ====

	// p0 = lo(a_lo) * lo(b_lo)
	lda	5,s		// a_lo_L
	ldb	9,s		// b_lo_L
	mul			// D = p0H:p0L
	std	2,x		// result_lo = p0

	// p1 = hi(a_lo) * lo(b_lo) -- add shifted left 8
	lda	4,s		// a_lo_H
	ldb	9,s		// b_lo_L
	mul			// D = p1H:p1L
	addb	2,x		// result_lo_H += p1L
	stb	2,x
	adca	#0
	sta	1,x		// result_hi_L = p1H + carry
	clr	,x		// result_hi_H = 0

	// p2 = lo(a_lo) * hi(b_lo) -- add shifted left 8
	lda	5,s		// a_lo_L
	ldb	8,s		// b_lo_H
	mul			// D = p2H:p2L
	addb	2,x		// result_lo_H += p2L
	stb	2,x
	adca	1,x		// result_hi_L += p2H + carry
	sta	1,x
	ldb	#0
	adcb	,x		// result_hi_H += carry
	stb	,x

	// p3 = hi(a_lo) * hi(b_lo) -- add shifted left 16
	lda	4,s		// a_lo_H
	ldb	8,s		// b_lo_H
	mul			// D = p3H:p3L
	addb	1,x		// result_hi_L += p3L
	stb	1,x
	adca	,x		// result_hi_H += p3H + carry
	sta	,x

	// ==== Step 2: a_hi * b_lo (truncated 16) -> add to result_hi ====

	lda	3,s		// a_hi_L
	ldb	9,s		// b_lo_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	2,s		// a_hi_H
	ldb	9,s		// b_lo_L
	mul
	addb	,x
	stb	,x

	lda	3,s		// a_hi_L
	ldb	8,s		// b_lo_H
	mul
	addb	,x
	stb	,x

	// ==== Step 3: a_lo * b_hi (truncated 16) -> add to result_hi ====

	lda	5,s		// a_lo_L
	ldb	7,s		// b_hi_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	4,s		// a_lo_H
	ldb	7,s		// b_hi_L
	mul
	addb	,x
	stb	,x

	lda	5,s		// a_lo_L
	ldb	6,s		// b_hi_H
	mul
	addb	,x
	stb	,x

	rts