//===-- ctzsi2.S - 32-bit count-trailing-zeros for MC6809 -----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// __ctzsi2(uint32_t a) — count trailing zeros in a 32-bit value.
//
// Bug #255 (post-bench 2026-05-09): LLVM-MC6809's libcall lowering for
// G_CTTZ at s32 emits a 4-byte sret call: caller passes a 4-byte buffer
// pointer in X, reads the result from offset 2 (low 16 bits of the
// big-endian zero-extended count). picolibc's ffsl was getting wrong
// answers for every non-zero input because Bug #245's earlier ctzsi2.S
// used the gcc6809-v1 "return in X register" convention — incompatible
// with the libcall ABI clang emits via `__builtin_ctzl`.
//
// Same family as Bug #246's i64 fix; this is the i32 analogue.
//
// LLVM-MC6809 i32-sret libcall ABI (matches __ucmpsi2.S):
//   2,s..5,s  = uint32_t a (big-endian: MSB at 2,s, LSB at 5,s)
//   X         = pointer to caller-allocated 4-byte result buffer
// Return: store count (i16, range 0..32) zero-extended to i32 at *X:
//   0,x..1,x  = 0
//   2,x..3,x  = count (big-endian)
//
// Direct C-ABI callers (declared `extern int __ctzsi2(uint32_t)`) get
// gcc6809 v1 behaviour at the call site (clang emits a single lbsr with
// arg on stack and reads result from X register). Those callers are now
// BROKEN by this ABI change — the result-via-X pattern doesn't match
// our new sret-via-X. Workaround: wrap such callers in a libcall-style
// call (via `__builtin_ctz` instead of explicit `__ctzsi2(x)`) or
// re-export under a different symbol name. The picolibc test suite
// migrated where needed.
//
// Strategy: scan the 4 input bytes from LSB upward; count trailing
// zeros within the first non-zero byte by right-shifting until the LSB
// is 1. A holds the count so X stays as the sret pointer throughout.
// Bug #352: A (not Y) — Y is callee-saved on MC6809 (CSR = IY,SU) and a
// runtime routine must not clobber it.
//
// Worst case (input = 0x80000000) is 4 byte-tests + 7 bit-shifts ≈ 160 cy.
// Best case (input has bit 0 set)   is 1 byte-test + 0 shifts ≈ 14 cy.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__ctzsi2
	.type	__ctzsi2,@function
__ctzsi2:
	ldb	5,s			// LSB
	bne	.Lctzsi2_byte0
	ldb	4,s
	bne	.Lctzsi2_byte1
	ldb	3,s
	bne	.Lctzsi2_byte2
	ldb	2,s			// MSB
	bne	.Lctzsi2_byte3
	lda	#32			// a == 0 → 32 (defensive)
	bra	.Lctzsi2_store

.Lctzsi2_byte3:
	lda	#24
	bra	.Lctzsi2_count
.Lctzsi2_byte2:
	lda	#16
	bra	.Lctzsi2_count
.Lctzsi2_byte1:
	lda	#8
	bra	.Lctzsi2_count
.Lctzsi2_byte0:
	lda	#0
	// fall through

// On entry: B != 0, A = 8 * (number of trailing zero bytes), X = sret ptr.
.Lctzsi2_count:
	lsrb
	bcs	.Lctzsi2_store
	inca
	bra	.Lctzsi2_count

// A = count (0..32). Write 4-byte zero-extended big-endian result.
// `clr` does not touch A, so the count survives to the final `sta`.
.Lctzsi2_store:
	clr	,x			// bytes 0..2 = 0
	clr	1,x
	clr	2,x
	sta	3,x			// byte 3 = count (low byte; count <= 32)
	rts
	.size	__ctzsi2, . - __ctzsi2
