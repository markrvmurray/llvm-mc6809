//===-- clzsi2.S - 32-bit count-leading-zeros for MC6809 ------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __clzsi2(uint32_t a) — count leading zeros in a 32-bit value.
//
// Bug #225 (original): __clzdi2.c shipped from compiler-rt's generic
// build references __clzsi2 internally; without this stub, executables
// pulling in __clzdi2 (e.g. via __builtin_clzll) failed to link.
//
// Bug #255 (post-bench 2026-05-09 — same family as the ctzsi2 fix):
// LLVM-MC6809's libcall lowering for G_CTLZ at s32 emits a 4-byte sret
// call (caller passes 4-byte buffer pointer in X, reads result from
// offset 2). The original Bug #225 .S used gcc6809-v1 "return in X
// register" — incompatible with the libcall ABI clang emits via
// `__builtin_clz`. Latent for months because no shipping picolibc test
// exercised __builtin_clz at i32 width; surfaced when the test-ffs
// FAIL diagnosis revealed the same pattern at __ctzsi2.
//
// LLVM-MC6809 i32-sret libcall ABI (matches __ucmpsi2.S / __ctzsi2.S):
//   2,s..5,s  = uint32_t a (big-endian: MSB at 2,s, LSB at 5,s)
//   X         = pointer to caller-allocated 4-byte result buffer
// Return: store count (i16, range 0..32) zero-extended to i32 at *X:
//   0,x..1,x  = 0
//   2,x..3,x  = count (big-endian)
//
// Direct C-ABI callers (declared `extern int __clzsi2(uint32_t)`) are
// BROKEN by this ABI change — same caveat as ctzsi2.S.
//
// Strategy: scan the 4 input bytes from MSB downward; count leading
// zeros within the first non-zero byte by left-shifting until the MSB
// is 1. A holds the count so X stays as the sret pointer throughout.
// Bug #352: A (not Y) — Y is callee-saved on MC6809 (CSR = IY,SU) and a
// runtime routine must not clobber it.
//
// Worst case (input = 0x00000001) is 4 byte-tests + 7 bit-shifts ≈ 160 cy.
// Best case (high bit set)         is 1 byte-test + 0 shifts ≈ 14 cy.
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
	lda	#32			// a == 0 → 32 leading zeros
	bra	.Lclzsi2_store

.Lclzsi2_byte3:
	lda	#24
	bra	.Lclzsi2_count
.Lclzsi2_byte2:
	lda	#16
	bra	.Lclzsi2_count
.Lclzsi2_byte1:
	lda	#8
	bra	.Lclzsi2_count
.Lclzsi2_byte0:
	lda	#0
	// fall through

// On entry: B != 0, A = 8 * (number of leading zero bytes), X = sret ptr.
.Lclzsi2_count:
	tstb
	bmi	.Lclzsi2_store
	lslb
	inca
	bra	.Lclzsi2_count

// A = count (0..32). Write 4-byte zero-extended big-endian result.
// `clr` does not touch A, so the count survives to the final `sta`.
.Lclzsi2_store:
	clr	,x			// bytes 0..2 = 0
	clr	1,x
	clr	2,x
	sta	3,x			// byte 3 = count (low byte; count <= 32)
	rts
	.size	__clzsi2, . - __clzsi2
