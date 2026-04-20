//===-- memops.S - __memset for MC6809 ---------------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __memset -- the LLVM intrinsic version of memset for MC6809.
//
// memset(void *s, int c, size_t n)    -- C library, c is 16-bit int
// __memset(void *s, char c, size_t n) -- LLVM intrinsic, c is 8-bit
//
// These differ in how c is passed on the stack: memset passes c as a
// 2-byte int (low byte at 3,S), while __memset passes c as a 1-byte
// char (value at 2,S). The backend emits __memset for llvm.memset
// intrinsic calls and memset for explicit C calls.
//
// We only provide __memset here. The plain memset is provided by the
// C library (picolibc, newlib, etc.) since it's a standard libc
// function. Standalone programs without a libc use the test runtime's
// memops.inc.
//
// gcc6809 ABI v1: IX = s, 2,S = c (1 byte), 3,S = n (2 bytes).
// Returns IX = s. Clobbers: D.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__memset
	.type	__memset,@function
__memset:
	pshs	x		// save s for return
	ldb	4,s		// c byte (+2 saved X, +2 retaddr)
	ldy	5,s		// n (+2 saved X, +2 retaddr, +1 c)
	beq	.L__memset_done
.L__memset_loop:
	stb	,x+
	leay	-1,y
	bne	.L__memset_loop
.L__memset_done:
	puls	x,pc
