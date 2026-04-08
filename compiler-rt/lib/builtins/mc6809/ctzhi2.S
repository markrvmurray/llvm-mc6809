//===-- ctzhi2.S - 16-bit count-trailing-zeros for MC6809 ------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// int __ctzhi2(uint16_t a) — count trailing zeros in a 16-bit value.
// Returns the number of trailing zero bits in a, or 16 if a is zero.
//
// gcc6809 ABI v1: first i16 arg in X, return i16 in X. Caller-saved
// regs: A, B, X, CC, DP. Y, U, S are callee-saved. This function
// only touches D and X, both caller-saved, so no save/restore needed.
//
// The bit-test loop terminates as soon as the lowest set bit is found,
// so typical inputs (with set bits in the low half) execute only a few
// iterations. Worst case is 16 iterations for input 0x8000.
//
// This libcall exists because the upstream LegalizerHelper SWAR lowering
// of G_CTPOP s16 (which CTTZ chains through) needs too many simultaneous
// i16 temps for the MC6809's tiny register file — see bug #73 and the
// regalloc-strategy discussion. A hand-written loop fits in a few bytes
// and uses exactly D and X, leaving the caller's registers untouched.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__ctzhi2
	.type	__ctzhi2,@function
__ctzhi2:
	cmpx	#0
	bne	.Lctzhi2_nonzero
	ldx	#16			// a == 0 → 16 trailing zeros (per C convention)
	rts
.Lctzhi2_nonzero:
	tfr	x,d			// D = a, will be shifted right
	ldx	#0			// X = trailing-zero counter
.Lctzhi2_loop:
	bitb	#1			// test the current low bit of D
	bne	.Lctzhi2_done
	lsra				// D >>= 1  (lsra + rorb is 16-bit logical shift right)
	rorb
	leax	1,x
	bra	.Lctzhi2_loop
.Lctzhi2_done:
	rts
