//===-- popcounthi2.S - 16-bit popcount for MC6809 ------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// int __popcounthi2(uint16_t a) — popcount of a 16-bit value.
//
// Bug #242 (partial): non-LTO HD6309 picolibc memalign / posix-memalign /
// hcreate_r reference __popcounthi2 (probably from `(align & (align-1))`-style
// power-of-two checks lowered through G_CTPOP s16) but compiler-rt has no
// upstream popcounthi2.c and no hand-written stub for it on mc6809. Linker
// fails: `undefined symbol: __popcounthi2`. This file plugs that gap.
//
// gcc6809 ABI v1 (matches __ctzhi2.S):
//   X = uint16_t a (16-bit input)
// Return: int (i16) in X (range 0..16)
//
// Strategy: split X into high/low bytes, popcount each via 8 unrolled
// `lsrb; adca #0` (drop LSB into carry, accumulate in A). Sum the two
// per-byte counts in Y, write Y back to X.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__popcounthi2
	.type	__popcounthi2,@function
__popcounthi2:
	pshs	y			// save Y (callee-saved)
	tfr	x,d			// D = a; A = high byte, B = low byte
	pshs	a			// stash high byte on stack
	ldy	#0			// total = 0
	bsr	.Lpopcounthi2_byte	// popcount(low) → A; clobber B
	leay	a,y
	ldb	,s			// B = saved high byte
	bsr	.Lpopcounthi2_byte	// popcount(high) → A
	leay	a,y
	leas	1,s			// release stashed byte
	tfr	y,x			// X = total (return value)
	puls	y			// restore Y
	rts

// Subroutine: A = popcount(B); on exit B = 0.
.Lpopcounthi2_byte:
	clra
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	lsrb
	adca	#0
	rts

	.size	__popcounthi2, . - __popcounthi2
