//===-- paritydi2.S - 64-bit parity for MC6809 ----------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __paritydi2(uint64_t a) — parity (popcount mod 2) of a 64-bit value.
//
// Bug #246: i64-sret libcall ABI. See ctzdi2.S header.
//
//   2,s..9,s  = uint64_t a (big-endian: MSB at 2,s, LSB at 9,s)
//   X         = 8-byte sret buffer
// Return: parity (0 or 1) zero-extended to i64 at *X.
//
// Strategy: XOR all 8 bytes into one byte, then fold that byte's bits
// pairwise (b ^= b >> 4; b ^= b >> 2; b ^= b >> 1). Low bit is parity.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__paritydi2
	.type	__paritydi2,@function
__paritydi2:
	ldb	2,s
	eorb	3,s
	eorb	4,s
	eorb	5,s
	eorb	6,s
	eorb	7,s
	eorb	8,s
	eorb	9,s
	// B = XOR of all input bytes. Collapse:
	pshs	b
	lsrb
	lsrb
	lsrb
	lsrb
	eorb	,s		// B = b ^ (b >> 4)
	stb	,s
	lsrb
	lsrb
	eorb	,s		// B = (above) ^ ((above) >> 2)
	stb	,s
	lsrb
	eorb	,s+		// B = (above) ^ ((above) >> 1); dealloc
	andb	#1		// B = parity
	// Write 8-byte zero-extended big-endian result.
	pshs	b
	clra
	clrb
	std	,x
	std	2,x
	std	4,x
	puls	b		// B = parity, A = 0 (still)
	std	6,x		// bytes 6..7 = (0, parity)
	rts
	.size	__paritydi2, . - __paritydi2
