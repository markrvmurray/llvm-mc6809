//===-- clzdi2.S - 64-bit count-leading-zeros for MC6809 ------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __clzdi2(uint64_t a) — count leading zeros in a 64-bit value.
//
// Bug #246: same i64-sret ABI mismatch as ctzdi2.S — see that file's
// header. compiler-rt's C `__clzdi2.c` returns int via X register, but
// LLVM-MC6809 lowers `__builtin_clzll` expecting i64-sret. Hand-written
// replacement.
//
// LLVM-MC6809 i64-sret libcall ABI:
//   2,s..9,s  = uint64_t a  (big-endian: MSB at 2,s, LSB at 9,s)
//   X         = pointer to caller-allocated 8-byte result buffer
// Return: store count (i16, range 0..64) zero-extended to i64 at *X:
//   0,x..5,x  = 0
//   6,x..7,x  = count (big-endian)
//
// Strategy: scan the 8 input bytes from MSB downward; count leading zeros
// within the first non-zero byte by left-shifting until the MSB is 1.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__clzdi2
	.type	__clzdi2,@function
__clzdi2:
	ldb	2,s			// MSB byte
	bne	.Lclzdi2_byte7
	ldb	3,s
	bne	.Lclzdi2_byte6
	ldb	4,s
	bne	.Lclzdi2_byte5
	ldb	5,s
	bne	.Lclzdi2_byte4
	ldb	6,s
	bne	.Lclzdi2_byte3
	ldb	7,s
	bne	.Lclzdi2_byte2
	ldb	8,s
	bne	.Lclzdi2_byte1
	ldb	9,s			// LSB byte
	bne	.Lclzdi2_byte0
	lda	#64			// a == 0 → 64 (defensive)
	bra	.Lclzdi2_store

.Lclzdi2_byte0:
	lda	#56
	bra	.Lclzdi2_count
.Lclzdi2_byte1:
	lda	#48
	bra	.Lclzdi2_count
.Lclzdi2_byte2:
	lda	#40
	bra	.Lclzdi2_count
.Lclzdi2_byte3:
	lda	#32
	bra	.Lclzdi2_count
.Lclzdi2_byte4:
	lda	#24
	bra	.Lclzdi2_count
.Lclzdi2_byte5:
	lda	#16
	bra	.Lclzdi2_count
.Lclzdi2_byte6:
	lda	#8
	bra	.Lclzdi2_count
.Lclzdi2_byte7:
	lda	#0
	// fall through

// On entry: B != 0, A = 8 * (number of leading zero bytes), X = sret ptr.
// Bug #352: count is in A (caller-saved), not Y — Y is callee-saved
// (MC6809 CSR = IY,SU) and a runtime routine must preserve it.
.Lclzdi2_count:
	tstb
	bmi	.Lclzdi2_store
	lslb
	inca
	bra	.Lclzdi2_count

// A = count (0..64). Write 8-byte zero-extended big-endian result.
// `clr` does not touch A, so the count survives to the final `sta`.
.Lclzdi2_store:
	clr	,x
	clr	1,x
	clr	2,x
	clr	3,x
	clr	4,x
	clr	5,x
	clr	6,x
	sta	7,x			// byte 7 = count (low byte; count <= 64)
	rts
	.size	__clzdi2, . - __clzdi2
