//===-- umulohi4.S - 16-bit unsigned multiply with overflow ---*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint16_t __umulohi4(uint16_t a, uint16_t b, int *overflow);
//
// The low 16 bits of a*b, which __builtin_mul_overflow hands back whether
// or not it overflowed, and *overflow = 1 when the product needs more than
// 16 bits, else 0. The compiler calls this for a 16-bit unsigned multiply
// with overflow -- calloc's size check, fread, fwrite -- and for the cross
// product of a 32-bit one.
//
// With a = ah:al and b = bh:bl, three MULs:
//   cross   = ah*bl + al*bh      (one term is zero unless both high bytes
//                                 are nonzero, which overflows anyway)
//   product = al*bl + (cross << 8)
//   overflow when ah and bh are both nonzero, when cross needs more than
//   a byte, or when adding it into the high byte of al*bl carries.
//
// Arguments: a in X, b at 2,S, overflow at 4,S (above the return address).
// Result in X. Clobbers D and CC; the same code on both processors.
//
//===----------------------------------------------------------------------===//

	.section .text.__umulohi4,"ax",@progbits
	.globl	__umulohi4
	.type	__umulohi4,@function
__umulohi4:
	pshs	x		// 0,S ah  1,S al  2,S return  4,S bh  5,S bl  6,S overflow
	lda	,s		// ah
	ldb	5,s		// bl
	mul			// D = ah*bl
	pshs	d		// cross at 0,S; everything else moves down two
	lda	3,s		// al
	ldb	6,s		// bh
	mul			// D = al*bh
	addd	,s		// D = cross
	std	,s
	lda	3,s		// al
	ldb	7,s		// bl
	mul			// D = al*bl
	adda	1,s		// add the cross product into the high byte; C = carried
	tfr	d,x		// X = the product (TFR leaves C alone)
	ldd	#0		// LDD leaves C alone too
	rolb			// B = the carry
	tst	,s		// does the cross product need more than a byte?
	beq	1f
	ldb	#1
1:	tst	2,s		// are both high bytes nonzero?
	beq	2f
	tst	6,s
	beq	2f
	ldb	#1
2:	std	[8,s]		// *overflow, with A = 0
	leas	4,s
	rts
	.size	__umulohi4, .-__umulohi4
