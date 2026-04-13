;;; __mulsi3 — 32-bit unsigned multiply (truncated to 32 bits)
;;;
;;; ABI (gcc6809 sret-style, ABI #4):
;;;   IX        = sret pointer (caller-allocated 4-byte buffer)
;;;   2,s..3,s  = a_hi
;;;   4,s..5,s  = a_lo
;;;   6,s..7,s  = b_hi
;;;   8,s..9,s  = b_lo
;;; Result (hi at IX+0, lo at IX+2; big-endian) is written through IX.
;;; Clobbers: D. Preserves: Y, U.
;;;
;;; Algorithm (schoolbook, truncated to 32 bits):
;;;   full32 = a_lo * b_lo           (16x16→32 via 4 8-bit MULs)
;;;   cross1 = a_hi * b_lo           (16x16→16 truncated, 3 MULs)
;;;   cross2 = a_lo * b_hi           (16x16→16 truncated, 3 MULs)
;;;   result_lo = lo16(full32)
;;;   result_hi = hi16(full32) + cross1 + cross2
;;;
;;; The 32-bit result accumulator lives at [IX..IX+3] (the caller's
;;; sret buffer), which doubles as our scratch workspace. No stack
;;; frame is needed since we don't touch any callee-saved registers.

	.globl	__mulsi3
	.type	__mulsi3,@function
__mulsi3:
	;; ==== Step 1: a_lo * b_lo → 32-bit result at [IX..IX+3] ====

	;; p0 = lo(a_lo) * lo(b_lo)
	lda	5,s		; a_lo_L
	ldb	9,s		; b_lo_L
	mul			; D = p0H:p0L
	std	2,x		; result_lo = p0

	;; p1 = hi(a_lo) * lo(b_lo)  — add shifted left 8
	lda	4,s		; a_lo_H
	ldb	9,s		; b_lo_L
	mul			; D = p1H:p1L
	addb	2,x		; result_lo_H += p1L
	stb	2,x
	adca	#0
	sta	1,x		; result_hi_L = p1H + carry
	clr	,x		; result_hi_H = 0

	;; p2 = lo(a_lo) * hi(b_lo)  — add shifted left 8
	lda	5,s		; a_lo_L
	ldb	8,s		; b_lo_H
	mul			; D = p2H:p2L
	addb	2,x		; result_lo_H += p2L
	stb	2,x
	adca	1,x		; result_hi_L += p2H + carry
	sta	1,x
	ldb	#0
	adcb	,x		; result_hi_H += carry
	stb	,x

	;; p3 = hi(a_lo) * hi(b_lo)  — add shifted left 16
	lda	4,s		; a_lo_H
	ldb	8,s		; b_lo_H
	mul			; D = p3H:p3L
	addb	1,x		; result_hi_L += p3L
	stb	1,x
	adca	,x		; result_hi_H += p3H + carry
	sta	,x

	;; ==== Step 2: a_hi * b_lo (truncated 16) → add to result_hi ====

	lda	3,s		; a_hi_L
	ldb	9,s		; b_lo_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	2,s		; a_hi_H
	ldb	9,s		; b_lo_L
	mul
	addb	,x
	stb	,x

	lda	3,s		; a_hi_L
	ldb	8,s		; b_lo_H
	mul
	addb	,x
	stb	,x

	;; ==== Step 3: a_lo * b_hi (truncated 16) → add to result_hi ====

	lda	5,s		; a_lo_L
	ldb	7,s		; b_hi_L
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	4,s		; a_lo_H
	ldb	7,s		; b_hi_L
	mul
	addb	,x
	stb	,x

	lda	5,s		; a_lo_L
	ldb	6,s		; b_hi_H
	mul
	addb	,x
	stb	,x

	rts
