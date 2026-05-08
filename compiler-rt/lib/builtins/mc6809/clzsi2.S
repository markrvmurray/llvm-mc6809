//===-- clzsi2.S - 32-bit count-leading-zeros for MC6809 ------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// int __clzsi2(uint32_t a) — count leading zeros in a 32-bit value.
//
// gcc6809 ABI v1:
//   2,s..5,s  = uint32_t a (big-endian: MSB at 2,s, LSB at 5,s)
// Return: int in X
//
// The upstream clzsi2.c lowering hits register pressure on the 6809 (see
// CMakeLists.txt:893-896 — clzsi2/ctzsi2/ffssi2/paritysi2/popcountsi2 are
// excluded from mc6809_SOURCES). But __clzdi2.c IS included and recurses
// into __clzsi2 — so the libcall must exist (Bug #225).
//
// Strategy: scan the 4 input bytes from MSB downward to find the first
// non-zero byte, then count leading zeros within that byte by left-shifting
// until the high bit is set. Each iteration is 8-bit, so register pressure
// is minimal: B holds the current byte, X is the running count.
//
// Worst case (input = 0x00000001) is 4 byte-tests + 7 bit-shifts ≈ 150 cy.
// Best case (high bit set) is 1 byte-test + 0 shifts ≈ 12 cy.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__clzsi2
	.type	__clzsi2,@function
__clzsi2:
	ldb	2,s			// MSB
	bne	.Lclzsi2_byte0
	ldb	3,s
	bne	.Lclzsi2_byte1
	ldb	4,s
	bne	.Lclzsi2_byte2
	ldb	5,s			// LSB
	bne	.Lclzsi2_byte3
	ldx	#32			// a == 0 → 32 leading zeros
	rts

.Lclzsi2_byte3:
	ldx	#24
	bra	.Lclzsi2_count
.Lclzsi2_byte2:
	ldx	#16
	bra	.Lclzsi2_count
.Lclzsi2_byte1:
	ldx	#8
	bra	.Lclzsi2_count
.Lclzsi2_byte0:
	ldx	#0
	// fall through

// On entry: B != 0, X = 8 * (number of leading zero bytes).
// Shift B left until its MSB is 1, incrementing X each iteration.
.Lclzsi2_count:
	tstb
	bmi	.Lclzsi2_done
	lslb
	leax	1,x
	bra	.Lclzsi2_count
.Lclzsi2_done:
	rts
	.size	__clzsi2, . - __clzsi2
