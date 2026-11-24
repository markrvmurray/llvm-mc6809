//===-- popcountsi2.S - 32-bit population count for MC6809 ----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __popcountsi2(uint32_t a) — number of set bits in a 32-bit value.
//
// The generic popcountsi2.c is left out of this target's builtins with the
// other 32-bit functions; this is its replacement. G_CTPOP at s32
// (__builtin_popcountl, C23 stdc_count_ones/zeros on unsigned long) reaches
// it.
//
// i32-sret libcall ABI (as __ctzsi2.S):
//   2,s..5,s  = uint32_t a (big-endian: MSB at 2,s, LSB at 5,s)
//   X         = pointer to caller-allocated 4-byte result buffer
// Return: count (0..32) zero-extended to i32 at *X.
//
// Strategy (as __popcountdi2.S): per-byte popcount via 8 unrolled
// `lsrb; adca #0`, accumulated in Y. X stays as the sret pointer.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__popcountsi2
	.type	__popcountsi2,@function
	.cfi_sections .debug_frame
__popcountsi2:
	.cfi_startproc
	// Y is callee-saved: push it, which moves the argument from 2,s..5,s
	// to 4,s..7,s; the bsr/rts pairs balance, so the offsets hold.
	pshs	y
	.cfi_adjust_cfa_offset 2
	.cfi_offset y, -4
	ldy	#0			// running total

	ldb	4,s			// MSB
	bsr	.Lpopsi2_byte
	ldb	5,s
	bsr	.Lpopsi2_byte
	ldb	6,s
	bsr	.Lpopsi2_byte
	ldb	7,s			// LSB
	bsr	.Lpopsi2_byte

	// Y = total (0..32): store it zero-extended to 32 bits.
	ldd	#0
	std	,x
	sty	2,x
	puls	y			// restore caller's Y
	.cfi_adjust_cfa_offset -2
	.cfi_restore y
	rts

// Subroutine: Y += popcount(B). Clobbers A, B; preserves X.
.Lpopsi2_byte:
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
	leay	a,y			// Y += A (A in 0..8)
	rts

	.cfi_endproc
	.size	__popcountsi2, . - __popcountsi2
