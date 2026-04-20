//===-- shiftqi3.S - 8-bit shifts for MC6809 ---------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// 8-bit variable shift library functions.
//
// All routines: B = value, 2,S = shift amount (1-byte stack slot,
// gcc6809 calling convention). Return addr at 0,S..1,S, then the
// shift-amount byte at 2,S.
// Returns: B = result. Clobbers: A. Preserves: X, Y, U.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

// __ashlqi3 -- 8-bit left shift
	.globl	__ashlqi3
	.type	__ashlqi3,@function
__ashlqi3:
	lda	2,s		// shift count
	beq	.Lshl8_done
.Lshl8_loop:
	aslb
	deca
	bne	.Lshl8_loop
.Lshl8_done:
	rts

// __lshrqi3 -- 8-bit logical right shift
	.globl	__lshrqi3
	.type	__lshrqi3,@function
__lshrqi3:
	lda	2,s
	beq	.Llsr8_done
.Llsr8_loop:
	lsrb
	deca
	bne	.Llsr8_loop
.Llsr8_done:
	rts

// __ashrqi3 -- 8-bit arithmetic right shift
	.globl	__ashrqi3
	.type	__ashrqi3,@function
__ashrqi3:
	lda	2,s
	beq	.Lasr8_done
.Lasr8_loop:
	asrb
	deca
	bne	.Lasr8_loop
.Lasr8_done:
	rts
