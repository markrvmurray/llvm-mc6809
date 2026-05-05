//===-- mulsi3.S - 32-bit multiply for MC6809 --------------*- ASM -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// uint32_t __mulsi3(uint32_t a, uint32_t b)
//
// 32-bit unsigned multiply, truncated to 32 bits.
//
// HD6309 path (`__6309__` defined, bug #161 Phase 1): schoolbook long
// multiplication using THREE 16-bit MULDs (one for the full a_lo*b_lo
// product, two for the low halves of the cross terms a_hi*b_lo and
// a_lo*b_hi). The a_hi*b_hi term is dead — its product would be
// shifted left 32 bits, beyond the truncated result. Each MULD
// produces Q = D:W (D = high 16, W = low 16).
//
// 6809 path: schoolbook with TEN 8-bit MULs. Each i16 partial product
// is broken into four 8-bit MULs.
//
// gcc6809 ABI v1 (sret-style):
//   IX        = sret pointer (caller-allocated 4-byte buffer)
//   2,S..3,S  = a_hi
//   4,S..5,S  = a_lo
//   6,S..7,S  = b_hi
//   8,S..9,S  = b_lo
// Result (hi at IX+0, lo at IX+2; big-endian) is written through IX.
// Clobbers: D (and W on HD6309). Preserves: Y, U.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits

	.globl	__mulsi3
	.type	__mulsi3,@function
__mulsi3:
#ifdef __6309__
	// Bug #161 Phase 1: HD6309 schoolbook with MULD partial products.
	//
	// MULD on HD6309 is SIGNED 16x16 → 32. For low-half-of-product,
	// signed and unsigned multiplication agree, so the cross-term
	// contributions (a_hi*b_lo and a_lo*b_hi, of which we only use
	// the low 16) are correct as-is. For the full a_lo*b_lo product
	// (where we need both high and low 16), we apply the standard
	// unsigned-from-signed correction:
	//
	//   unsigned(x) = signed(x) + (x_bit15 ? 0x10000 : 0)   for i16 x
	//
	// so unsigned(a)*unsigned(b) - signed(a)*signed(b) =
	//   (a_bit15 ? b<<16 : 0) + (b_bit15 ? a<<16 : 0) + (both_bits15 ? 1<<32 : 0)
	//
	// Truncating to 32 bits drops the (1<<32) term; the remaining
	// corrections add b_lo (or a_lo) to result_hi when the
	// corresponding operand is "negative".

	// p_full = signed(a_lo) * signed(b_lo) (full 32-bit at IX..IX+3)
	ldd	4,s		// D = a_lo
	muld	8,s		// Q = signed(a_lo * b_lo)  (D:W)
	stq	,x		// IX[0..3] = D:W

	// Unsigned correction A: if a_lo's bit 15 is set, add b_lo to result_hi.
	tst	4,s		// CC.N = MSB of a_lo (the high byte at 4,s)
	bpl	1f		// skip if a_lo >= 0 (bit 15 clear)
	ldd	,x
	addd	8,s		// result_hi += b_lo
	std	,x
1:
	// Unsigned correction B: if b_lo's bit 15 is set, add a_lo to result_hi.
	tst	8,s		// CC.N = MSB of b_lo
	bpl	2f
	ldd	,x
	addd	4,s		// result_hi += a_lo
	std	,x
2:

	// p_cross1 = a_hi * b_lo, low 16 added to result_hi (IX[0..1]).
	// Cross-term high 16 would land at bits 48..32 — dropped.
	// Low 16 is the same for signed and unsigned, so MULD is fine here.
	ldd	2,s		// D = a_hi
	muld	8,s		// Q = a_hi * b_lo (D = hi16, W = lo16)
	tfr	w,d		// D = lo16 of cross term
	addd	,x		// D += result_hi
	std	,x		// result_hi = D

	// p_cross2 = a_lo * b_hi, low 16 added to result_hi.
	ldd	4,s		// D = a_lo
	muld	6,s		// Q = a_lo * b_hi (D = hi16, W = lo16)
	tfr	w,d		// D = lo16 of cross term
	addd	,x		// D += result_hi
	std	,x		// result_hi = D

	rts
#else
	// ==== Step 1: a_lo * b_lo -> 32-bit result at [IX..IX+3] ====

	// p0 = lo(a_lo) * lo(b_lo)
	lda	5,s		// a_lo_L
	ldb	9,s		// b_lo_L
	mul			// D = p0H:p0L
	std	2,x		// result_lo = p0

	// p1 = hi(a_lo) * lo(b_lo) -- add shifted left 8
	lda	4,s		// a_lo_H
	ldb	9,s		// b_lo_L
	mul			// D = p1H:p1L
	addb	2,x		// result_lo_H += p1L
	stb	2,x
	adca	#0
	sta	1,x		// result_hi_L = p1H + carry
	clr	,x		// result_hi_H = 0

	// p2 = lo(a_lo) * hi(b_lo) -- add shifted left 8
	lda	5,s		// a_lo_L
	ldb	8,s		// b_lo_H
	mul			// D = p2H:p2L
	addb	2,x		// result_lo_H += p2L
	stb	2,x
	adca	1,x		// result_hi_L += p2H + carry
	sta	1,x
	ldb	#0
	adcb	,x		// result_hi_H += carry
	stb	,x

	// p3 = hi(a_lo) * hi(b_lo) -- add shifted left 16
	lda	4,s		// a_lo_H
	ldb	8,s		// b_lo_H
	mul			// D = p3H:p3L
	addb	1,x		// result_hi_L += p3L
	stb	1,x
	adca	,x		// result_hi_H += p3H + carry
	sta	,x

	// ==== Step 2: a_hi * b_lo (truncated 16) -> add to result_hi ====

	lda	3,s		// a_hi_L
	ldb	9,s		// b_lo_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	2,s		// a_hi_H
	ldb	9,s		// b_lo_L
	mul
	addb	,x
	stb	,x

	lda	3,s		// a_hi_L
	ldb	8,s		// b_lo_H
	mul
	addb	,x
	stb	,x

	// ==== Step 3: a_lo * b_hi (truncated 16) -> add to result_hi ====

	lda	5,s		// a_lo_L
	ldb	7,s		// b_hi_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	4,s		// a_lo_H
	ldb	7,s		// b_hi_L
	mul
	addb	,x
	stb	,x

	lda	5,s		// a_lo_L
	ldb	6,s		// b_hi_H
	mul
	addb	,x
	stb	,x

	rts
#endif