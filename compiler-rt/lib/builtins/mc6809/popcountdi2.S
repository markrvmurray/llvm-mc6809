//===-- popcountdi2.S - 64-bit population count for MC6809 ----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __popcountdi2(uint64_t a) — number of set bits in a 64-bit value.
//
// Bug #246: i64-sret libcall ABI. See ctzdi2.S header.
//
//   2,s..9,s  = uint64_t a (big-endian: MSB at 2,s, LSB at 9,s)
//   X         = 8-byte sret buffer
// Return: count (i16, 0..64) zero-extended to i64 at *X.
//
// Strategy: per-byte popcount via 8 unrolled `lsrb; adca #0` — each shift
// drops the LSB into carry, and `adca #0` adds it to A. After 8 iterations
// A = popcount(byte). Accumulate the 8 per-byte counts in Y (i16). X
// stays as the sret pointer.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__popcountdi2
	.type	__popcountdi2,@function
__popcountdi2:
	// Bug #352: Y is callee-saved (MC6809 CSR = IY,SU).  This routine
	// uses Y as the i16 running total across the byte subroutine, so it
	// must save/restore the caller's Y.  The `pshs y` shifts the input
	// arguments down by 2 bytes (2,s..9,s -> 4,s..11,s); the bsr/rts to
	// .Lpopdi2_byte balances, so the offsets are stable across the loop.
	pshs	y
	ldy	#0			// running total

	ldb	4,s			// MSB (was 2,s)
	bsr	.Lpopdi2_byte
	ldb	5,s
	bsr	.Lpopdi2_byte
	ldb	6,s
	bsr	.Lpopdi2_byte
	ldb	7,s
	bsr	.Lpopdi2_byte
	ldb	8,s
	bsr	.Lpopdi2_byte
	ldb	9,s
	bsr	.Lpopdi2_byte
	ldb	10,s
	bsr	.Lpopdi2_byte
	ldb	11,s			// LSB (was 9,s)
	bsr	.Lpopdi2_byte

	// Y = total popcount (0..64). Write 8-byte zero-extended result.
	ldd	#0
	std	,x
	std	2,x
	std	4,x
	sty	6,x
	puls	y			// restore caller's Y
	rts

// Subroutine: A = popcount(B); on exit Y += A. Clobbers A, B; preserves X.
.Lpopdi2_byte:
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
	leay	a,y			// Y += A (A in 0..8, treated as signed)
	rts

	.size	__popcountdi2, . - __popcountdi2
