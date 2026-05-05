//===-- mulhi3.S - 16-bit multiply for MC6809 --------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint16_t __mulhi3(uint16_t a, uint16_t b)
//
// 16-bit unsigned multiply, truncated to 16 bits.
//
// HD6309 path (`__6309__` defined): single MULD instruction.
//   D = a, MULD operand = b → Q (= D:W) holds the 32-bit product.
//   We only need the low 16 (= W).
//
// 6809 path: schoolbook long multiplication using three 8-bit MUL
// instructions. p1 = al*bl is the full 16-bit; p2 = ah*bl and
// p3 = al*bh contribute only their low bytes (their high bytes
// would overflow the 16-bit truncated result).
//
// gcc6809 ABI v1: X = a, 2,S = b (after return addr). Returns X = result.
// Clobbers: D (and W on HD6309). Preserves: Y, U.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__mulhi3
	.type	__mulhi3,@function
__mulhi3:
#ifdef __6309__
	// Bug #161 Phase 1: HD6309 single-instruction path.
	tfr	x,d		// D = a
	muld	2,s		// Q = a * b  (D = hi16, W = lo16)
	tfr	w,x		// X = lo16 = a*b mod 2^16
	rts
#else
	pshs	x		// save a; S: [ah][al][retH][retL][bh][bl]
	//       0    1     2      3     4    5

	// Partial product 1: al * bl -> full 16-bit result
	lda	1,s		// al
	ldb	5,s		// bl
	mul			// D = al * bl
	pshs	d		// save [hi(al*bl), lo(al*bl)]
	// S: [carry][res_lo][ah][al][retH][retL][bh][bl]
	//      0       1      2    3    4      5     6    7

	// Partial product 2: ah * bl -> add low byte to carry
	lda	2,s		// ah
	ldb	7,s		// bl
	mul			// D = ah * bl
	addb	,s		// carry += lo(ah * bl)
	stb	,s		// save updated carry

	// Partial product 3: al * bh -> add low byte to carry
	lda	3,s		// al
	ldb	6,s		// bh
	mul			// D = al * bh
	addb	,s		// carry += lo(al * bh)

	// Compose result: D = result_hi:result_lo
	tfr	b,a		// A = result_hi
	ldb	1,s		// B = result_lo
	tfr	d,x		// X = result

	leas	4,s		// clean up (pshs d + pshs x)
	rts
#endif