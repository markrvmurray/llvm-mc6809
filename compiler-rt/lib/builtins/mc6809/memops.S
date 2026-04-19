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
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__memset
	.type	__memset,@function
__memset:
	pshs	x,y		// save x for return; preserve y (callee-saved per MC6809_CSR)
	// B already holds the fill byte (passed in register by the compiler)
	ldy	6,s		// n (+2 saved Y, +2 saved X, +2 retaddr)
	beq	.L__memset_done
.L__memset_loop:
	stb	,x+
	leay	-1,y
	bne	.L__memset_loop
.L__memset_done:
	puls	x,y,pc
