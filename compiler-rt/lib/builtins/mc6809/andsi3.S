//===-- andsi3.S - 32-bit bitwise AND for MC6809 -----------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint32_t __andsi3(uint32_t a, uint32_t b)
//
// 32-bit bitwise AND. Bug #210 round 5 routes G_AND i32 here on HD6309
// (and 6809) to avoid the in-line decomposition that creates AQc-class
// vregs in __call_hash et al.
//
// gcc6809 ABI v1 (sret-style):
//   IX        = sret pointer (caller-allocated 4-byte buffer)
//   2,S..3,S  = a_hi
//   4,S..5,S  = a_lo
//   6,S..7,S  = b_hi
//   8,S..9,S  = b_lo
// Result (hi at IX+0, lo at IX+2; big-endian) is written through IX.
// Clobbers: A. Preserves: B, D's high-meaningful, Y, U.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits
	.globl	__andsi3
	.type	__andsi3,@function
__andsi3:
	lda	2,s		// a byte 0 (MSB)
	anda	6,s		// & b byte 0
	sta	,x
	lda	3,s
	anda	7,s
	sta	1,x
	lda	4,s
	anda	8,s
	sta	2,x
	lda	5,s
	anda	9,s		// & b byte 3 (LSB)
	sta	3,x
	rts
	.size	__andsi3, . - __andsi3
