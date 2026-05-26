//===-- ctzdi2.S - 64-bit count-trailing-zeros for MC6809 -----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __ctzdi2(uint64_t a) — count trailing zeros in a 64-bit value.
//
// Bug #246: LLVM-MC6809's libcall lowering for `__builtin_ctzll` / G_CTTZ
// at s64 emits an i64-sret call: caller passes an 8-byte buffer pointer
// in X and reads the result from offset 6 (low 16 bits of the big-endian
// zero-extended count). compiler-rt's C `__ctzdi2.c` returns int via X
// register (gcc6809 v1), so the C-compiled body silently miscompiles
// every `__builtin_ctzll` user. Hand-written replacement below.
//
// LLVM-MC6809 i64-sret libcall ABI:
//   2,s..9,s  = uint64_t a  (big-endian: MSB at 2,s, LSB at 9,s)
//   X         = pointer to caller-allocated 8-byte result buffer
// Return: store count (i16, range 0..64) zero-extended to i64 at *X:
//   0,x..5,x  = 0
//   6,x..7,x  = count (big-endian)
//
// Strategy: scan the 8 input bytes from LSB upward (9,s down to 2,s) for
// the first non-zero byte; count trailing zeros within that byte by
// right-shifting until the LSB is 1. Y holds the count so X stays as
// the sret pointer.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__ctzdi2
	.type	__ctzdi2,@function
__ctzdi2:
	ldb	9,s			// LSB byte
	bne	.Lctzdi2_byte0
	ldb	8,s
	bne	.Lctzdi2_byte1
	ldb	7,s
	bne	.Lctzdi2_byte2
	ldb	6,s
	bne	.Lctzdi2_byte3
	ldb	5,s
	bne	.Lctzdi2_byte4
	ldb	4,s
	bne	.Lctzdi2_byte5
	ldb	3,s
	bne	.Lctzdi2_byte6
	ldb	2,s			// MSB byte
	bne	.Lctzdi2_byte7
	lda	#64			// a == 0 → 64 (defensive)
	bra	.Lctzdi2_store

.Lctzdi2_byte7:
	lda	#56
	bra	.Lctzdi2_count
.Lctzdi2_byte6:
	lda	#48
	bra	.Lctzdi2_count
.Lctzdi2_byte5:
	lda	#40
	bra	.Lctzdi2_count
.Lctzdi2_byte4:
	lda	#32
	bra	.Lctzdi2_count
.Lctzdi2_byte3:
	lda	#24
	bra	.Lctzdi2_count
.Lctzdi2_byte2:
	lda	#16
	bra	.Lctzdi2_count
.Lctzdi2_byte1:
	lda	#8
	bra	.Lctzdi2_count
.Lctzdi2_byte0:
	lda	#0
	// fall through

// On entry: B != 0, A = 8 * (number of trailing zero bytes), X = sret ptr.
// Bug #352: count is in A (caller-saved), not Y — Y is callee-saved
// (MC6809 CSR = IY,SU) and a runtime routine must preserve it.
.Lctzdi2_count:
	lsrb
	bcs	.Lctzdi2_store
	inca
	bra	.Lctzdi2_count

// A = count (0..64). Write 8-byte zero-extended big-endian result.
// `clr` does not touch A, so the count survives to the final `sta`.
.Lctzdi2_store:
	clr	,x
	clr	1,x
	clr	2,x
	clr	3,x
	clr	4,x
	clr	5,x
	clr	6,x
	sta	7,x			// byte 7 = count (low byte; count <= 64)
	rts
	.size	__ctzdi2, . - __ctzdi2
