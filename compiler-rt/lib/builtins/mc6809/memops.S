//===-- memops.S - __memset for MC6809 ---------------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __memset -- LLVM intrinsic memset for MC6809.
//
// The LLVM backend lowers llvm.memset to __memset. The intrinsic
// defines c as i8, which the MC6809 calling convention passes in
// register B (first i8 arg). The count n goes on the stack as i16.
//
// Bug #125: the previous version read c from the stack, but the
// compiler passes c in B. Reading from the stack got the high byte
// of the count (0x00) instead of the fill byte.
//
// Calling convention:
//   IX = s (destination pointer)
//   B  = c (fill byte, i8 in register)
//   2,S = n (count, i16 on stack)
// Returns IX = s. Clobbers: D. IY is callee-saved per MC6809_CSR.
//
// Stores a byte to make the count even, then a word to make it a
// multiple of four, then two words per pass: 24 cycles per four bytes,
// against 14 a byte for a byte loop.  One library serves the 6809 and
// the 6309, so this is 6809 code.
//
//===----------------------------------------------------------------------===//

	.section .text.__memset,"ax",@progbits

	.globl	__memset
	.type	__memset,@function
__memset:
	pshs	x,y		// save x for return; preserve y (callee-saved per MC6809_CSR)
	ldy	6,s		// Y = n (+2 saved Y, +2 saved X, +2 retaddr)
	beq	.L__memset_done
	tfr	b,a		// A = fill byte
	ldb	7,s		// B = low byte of n
	lsrb			// C = bit 0 of n
	bcc	.L__memset_even
	sta	,x+		// odd count: one byte first
	leay	-1,y
	beq	.L__memset_done
.L__memset_even:
	lsrb			// C = bit 1 of n
	tfr	a,b		// D = fill:fill (TFR leaves CC alone)
	bcc	.L__memset_loop
	std	,x++		// count not a multiple of four: one word first
	leay	-2,y
	beq	.L__memset_done
.L__memset_loop:
	std	,x++
	std	,x++
	leay	-4,y
	bne	.L__memset_loop
.L__memset_done:
	puls	x,y,pc
