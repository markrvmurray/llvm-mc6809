//===-- clzhi2.S - 16-bit count-leading-zeros for MC6809 ------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// int __clzhi2(uint16_t a) — count leading zeros in a 16-bit value.
// Returns the number of leading zero bits in a, or 16 if a is zero.
//
// This is what __builtin_clz calls, int being 16 bits here, and what the
// legalizer calls for any llvm.ctlz of 16 bits or fewer.
//
// gcc6809 ABI v1: first i16 arg in X, return i16 in X. Only D and X are
// touched, both caller-saved, so nothing needs saving.
//
// A zero high byte skips straight to the low one, which leaves at most seven
// shifts for any input.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__clzhi2
	.type	__clzhi2,@function
__clzhi2:
	cmpx	#0
	bne	.Lclzhi2_nonzero
	ldx	#16			// a == 0 → 16 leading zeros (matches __ctzhi2)
	rts
.Lclzhi2_nonzero:
	tfr	x,d			// D = a, will be shifted left
	ldx	#0			// X = leading-zero counter
	tsta
	bne	.Lclzhi2_loop
	tfr	b,a			// the high byte is zero: look at the low one
	ldx	#8
.Lclzhi2_loop:
	tsta				// N is the bit under test
	bmi	.Lclzhi2_done
	lsla				// A is nonzero, so it holds the first set bit
	leax	1,x
	bra	.Lclzhi2_loop
.Lclzhi2_done:
	rts
