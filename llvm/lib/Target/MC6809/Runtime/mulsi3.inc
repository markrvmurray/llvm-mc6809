;;; __mulsi3 — 32-bit unsigned multiply (truncated to 32 bits)
;;; Input:  X = a_hi, 2,s = a_lo, 4,s = b_hi, 6,s = b_lo
;;;         (all 16-bit, big-endian; a = a_hi:a_lo, b = b_hi:b_lo)
;;; Output: X = result_lo, result_hi written to caller stack at 2,s
;;; Clobbers: D
;;;
;;; Algorithm (schoolbook, truncated to 32 bits):
;;;   full32 = a_lo * b_lo           (16x16→32 via 4 8-bit MULs)
;;;   cross1 = a_hi * b_lo           (16x16→16 truncated, 3 MULs)
;;;   cross2 = a_lo * b_hi           (16x16→16 truncated, 3 MULs)
;;;   result_lo = lo16(full32)
;;;   result_hi = hi16(full32) + cross1 + cross2

	.globl	__mulsi3
	.type	__mulsi3,@function
__mulsi3:
	pshs	x		; save a_hi
	leas	-4,s		; allocate 4 bytes for result32
	; Frame (from S):
	;   S+0,1:   result32_hi  (workspace)
	;   S+2,3:   result32_lo  (workspace)
	;   S+4,5:   saved a_hi   (a_hi_H=4, a_hi_L=5)
	;   S+6,7:   return addr
	;   S+8,9:   a_lo         (a_lo_H=8, a_lo_L=9)
	;   S+10,11: b_hi         (b_hi_H=10, b_hi_L=11)
	;   S+12,13: b_lo         (b_lo_H=12, b_lo_L=13)

	;; ==== Step 1: a_lo * b_lo → 32-bit result ====

	;; p0 = lo(a_lo) * lo(b_lo)
	lda	9,s		; a_lo_L
	ldb	13,s		; b_lo_L
	mul			; D = p0H:p0L
	std	2,s		; result32_lo = p0 (initial value)

	;; p1 = hi(a_lo) * lo(b_lo)  — add shifted left 8
	lda	8,s		; a_lo_H
	ldb	13,s		; b_lo_L
	mul			; D = p1H:p1L
	addb	2,s		; result32_lo_H += p1L
	stb	2,s
	adca	#0		; A = p1H + carry
	sta	1,s		; result32_hi_L = p1H + carry
	clr	0,s		; result32_hi_H = 0

	;; p2 = lo(a_lo) * hi(b_lo)  — add shifted left 8
	lda	9,s		; a_lo_L
	ldb	12,s		; b_lo_H
	mul			; D = p2H:p2L
	addb	2,s		; result32_lo_H += p2L
	stb	2,s
	adca	1,s		; result32_hi_L += p2H + carry
	sta	1,s
	ldb	#0
	adcb	0,s		; result32_hi_H += carry
	stb	0,s

	;; p3 = hi(a_lo) * hi(b_lo)  — add shifted left 16
	lda	8,s		; a_lo_H
	ldb	12,s		; b_lo_H
	mul			; D = p3H:p3L
	addb	1,s		; result32_hi_L += p3L
	stb	1,s
	adca	0,s		; result32_hi_H += p3H + carry
	sta	0,s

	;; ==== Step 2: a_hi * b_lo (truncated 16) → add to result32_hi ====

	lda	5,s		; a_hi_L
	ldb	13,s		; b_lo_L
	mul
	addb	1,s		; result32_hi_L += lo(a_hi_L * b_lo_L)
	stb	1,s
	adca	0,s		; result32_hi_H += hi(...) + carry
	sta	0,s

	lda	4,s		; a_hi_H
	ldb	13,s		; b_lo_L
	mul
	addb	0,s		; result32_hi_H += lo(a_hi_H * b_lo_L)
	stb	0,s

	lda	5,s		; a_hi_L
	ldb	12,s		; b_lo_H
	mul
	addb	0,s		; result32_hi_H += lo(a_hi_L * b_lo_H)
	stb	0,s

	;; ==== Step 3: a_lo * b_hi (truncated 16) → add to result32_hi ====

	lda	9,s		; a_lo_L
	ldb	11,s		; b_hi_L
	mul
	addb	1,s		; result32_hi_L += lo(a_lo_L * b_hi_L)
	stb	1,s
	adca	0,s		; result32_hi_H += hi(...) + carry
	sta	0,s

	lda	8,s		; a_lo_H
	ldb	11,s		; b_hi_L
	mul
	addb	0,s		; result32_hi_H += lo(a_lo_H * b_hi_L)
	stb	0,s

	lda	9,s		; a_lo_L
	ldb	10,s		; b_hi_H
	mul
	addb	0,s		; result32_hi_H += lo(a_lo_L * b_hi_H)
	stb	0,s

	;; ==== Assemble result ====
	ldx	2,s		; X = result_lo
	ldd	0,s		; D = result_hi

	leas	6,s		; clean up: workspace(4) + saved_x(2)
	; S+0,1 = return addr
	; S+2,3 = caller's a_lo slot → write result_hi here
	std	2,s
	rts
