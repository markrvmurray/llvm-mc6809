//===-- ctzsi2.S - 32-bit count-trailing-zeros for MC6809 -----*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// int __ctzsi2(uint32_t a) — count trailing zeros in a 32-bit value.
//
// Precondition: a != 0 (matches the upstream contract from ctzsi2.c).
// On a == 0 the result is 32 (defensive — upstream is undefined here).
//
// gcc6809 ABI v1:
//   2,s..5,s  = uint32_t a (big-endian: MSB at 2,s, LSB at 5,s)
// Return: int in X
//
// NOTE: `__builtin_ctzll` lowers via the LLVM-MC6809 libcall path which
// uses a wider i64-sret return — the compiler-rt C-compiled `__ctzdi2`
// is incompatible with that convention. That ABI mismatch is filed
// separately; this file is the narrow `int __ctzsi2(uint32_t)` fix
// (Bug #245), exactly mirroring clzsi2.S so direct C-ABI callers work.
//
// Bug #245: __ctzdi2 (which IS shipped from compiler-rt's generic build)
// references __ctzsi2 internally; without this stub any executable that
// pulls in __ctzdi2 (e.g. via __builtin_ctzll) fails to link. The C
// version of __ctzsi2 is excluded from mc6809_SOURCES because the byte-
// level decomposition produces too much register pressure (CMakeLists.txt
// :893-896, alongside __clzsi2 which Bug #225 likewise replaced with a
// hand-written byte scan). This file mirrors clzsi2.S — same shape, MSB
// scan reversed to LSB scan, lslb→lsrb in the bit-count loop.
//
// Strategy: scan the 4 input bytes from LSB upward to find the first
// non-zero byte, then count trailing zeros within that byte by right-
// shifting until the LSB is 1. B holds the current byte, X the running
// count.
//
// Worst case (input = 0x80000000) is 4 byte-tests + 7 bit-shifts ≈ 150 cy.
// Best case (input has bit 0 set)   is 1 byte-test + 0 shifts ≈ 12 cy.
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
	ldx	#32			// a == 0 → 32 trailing zeros (defensive)
	rts

.Lctzsi2_byte3:
	ldx	#24
	bra	.Lctzsi2_count
.Lctzsi2_byte2:
	ldx	#16
	bra	.Lctzsi2_count
.Lctzsi2_byte1:
	ldx	#8
	bra	.Lctzsi2_count
.Lctzsi2_byte0:
	ldx	#0
	// fall through

// On entry: B != 0, X = 8 * (number of trailing zero bytes).
// Shift B right until the bit shifted out is 1; X counts zero shifts.
.Lctzsi2_count:
	lsrb
	bcs	.Lctzsi2_done
	leax	1,x
	bra	.Lctzsi2_count
.Lctzsi2_done:
	rts
	.size	__ctzsi2, . - __ctzsi2
