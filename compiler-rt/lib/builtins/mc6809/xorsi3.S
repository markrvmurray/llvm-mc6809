//===-- xorsi3.S - 32-bit bitwise XOR for MC6809 -----------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint32_t __xorsi3(uint32_t a, uint32_t b)
//
// 32-bit bitwise XOR. ABI as __andsi3.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits
	.globl	__xorsi3
	.type	__xorsi3,@function
__xorsi3:
	lda	2,s
	eora	6,s
	sta	,x
	lda	3,s
	eora	7,s
	sta	1,x
	lda	4,s
	eora	8,s
	sta	2,x
	lda	5,s
	eora	9,s
	sta	3,x
	rts
	.size	__xorsi3, . - __xorsi3
