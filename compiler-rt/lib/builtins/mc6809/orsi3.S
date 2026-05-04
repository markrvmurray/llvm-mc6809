//===-- orsi3.S - 32-bit bitwise OR for MC6809 -------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint32_t __orsi3(uint32_t a, uint32_t b)
//
// 32-bit bitwise OR. ABI as __andsi3.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits
	.globl	__orsi3
	.type	__orsi3,@function
__orsi3:
	lda	2,s
	ora	6,s
	sta	,x
	lda	3,s
	ora	7,s
	sta	1,x
	lda	4,s
	ora	8,s
	sta	2,x
	lda	5,s
	ora	9,s
	sta	3,x
	rts
	.size	__orsi3, . - __orsi3
