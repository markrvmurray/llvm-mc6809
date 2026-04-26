;;; muldi3.inc — 64-bit truncated multiply runtime entry point.
;;;
;;; Mirrors compiler-rt/lib/builtins/mc6809/muldi3.S so the
;;; execution-test mc6809rt.s can `.include` it without depending
;;; on a compiler-rt build. Update both files in sync.
;;;
;;; di_int __muldi3(di_int a, di_int b)
;;;   Returns the low 64 bits of a * b.
;;;
;;; Replaces the broken compiler-rt C version that triggered
;;; bug #173 (returned wrong answers for several i64 multiply
;;; inputs because its __muldsi3 32x32->64 helper has a partial-
;;; product alignment bug for big-endian + int=i16 + long=i32 +
;;; long long=i64).
;;;
;;; Calling convention (matches __mulsi3.inc, just one byte wider):
;;;   IX            = sret pointer (8-byte buffer, MSB at IX, LSB at IX+7)
;;;   2,S..9,S      = a (8 bytes, big-endian)
;;;   10,S..17,S    = b (8 bytes, big-endian)
;;; Clobbers D. Preserves Y, U.

	.globl __muldi3
	.type  __muldi3,@function
__muldi3:
	; Initialize result buffer to zero (8 bytes at IX..IX+7).
	clr	,x
	clr	1,x
	clr	2,x
	clr	3,x
	clr	4,x
	clr	5,x
	clr	6,x
	clr	7,x

	; ============================================================
	; i+j = 7  (low byte to IX+0; high byte off-end, dropped)
	; ============================================================
	; 8 partials: (0,7) (1,6) (2,5) (3,4) (4,3) (5,2) (6,1) (7,0)

	lda	2,s
	ldb	17,s
	mul
	addb	,x
	stb	,x

	lda	3,s
	ldb	16,s
	mul
	addb	,x
	stb	,x

	lda	4,s
	ldb	15,s
	mul
	addb	,x
	stb	,x

	lda	5,s
	ldb	14,s
	mul
	addb	,x
	stb	,x

	lda	6,s
	ldb	13,s
	mul
	addb	,x
	stb	,x

	lda	7,s
	ldb	12,s
	mul
	addb	,x
	stb	,x

	lda	8,s
	ldb	11,s
	mul
	addb	,x
	stb	,x

	lda	9,s
	ldb	10,s
	mul
	addb	,x
	stb	,x

	; ============================================================
	; i+j = 8  (low to IX+1, high to IX+0)
	; ============================================================

	lda	3,s
	ldb	17,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	4,s
	ldb	16,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	5,s
	ldb	15,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	6,s
	ldb	14,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	7,s
	ldb	13,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	8,s
	ldb	12,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	lda	9,s
	ldb	11,s
	mul
	addb	1,x
	stb	1,x
	adca	,x
	sta	,x

	; ============================================================
	; i+j = 9  (low to IX+2, high to IX+1, chain to IX+0)
	; ============================================================

	lda	4,s
	ldb	17,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	5,s
	ldb	16,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	6,s
	ldb	15,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	14,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	13,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	12,s
	mul
	addb	2,x
	stb	2,x
	adca	1,x
	sta	1,x
	ldb	,x
	adcb	#0
	stb	,x

	; ============================================================
	; i+j = 10  (low to IX+3, high to IX+2, chain to IX+1, IX+0)
	; ============================================================

	lda	5,s
	ldb	17,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	6,s
	ldb	16,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	15,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	14,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	13,s
	mul
	addb	3,x
	stb	3,x
	adca	2,x
	sta	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	; ============================================================
	; i+j = 11  (low to IX+4, high to IX+3, chain to IX+2, +1, +0)
	; ============================================================

	lda	6,s
	ldb	17,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	7,s
	ldb	16,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	15,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	14,s
	mul
	addb	4,x
	stb	4,x
	adca	3,x
	sta	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	; ============================================================
	; i+j = 12  (low to IX+5, high to IX+4, chain to IX+3..+0)
	; ============================================================

	lda	7,s
	ldb	17,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	8,s
	ldb	16,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	15,s
	mul
	addb	5,x
	stb	5,x
	adca	4,x
	sta	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	; ============================================================
	; i+j = 13  (low to IX+6, high to IX+5, chain to IX+4..+0)
	; ============================================================

	lda	8,s
	ldb	17,s
	mul
	addb	6,x
	stb	6,x
	adca	5,x
	sta	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	lda	9,s
	ldb	16,s
	mul
	addb	6,x
	stb	6,x
	adca	5,x
	sta	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	; ============================================================
	; i+j = 14  (low to IX+7, high to IX+6, chain to IX+5..+0)
	; ============================================================

	lda	9,s
	ldb	17,s
	mul
	addb	7,x
	stb	7,x
	adca	6,x
	sta	6,x
	ldb	5,x
	adcb	#0
	stb	5,x
	ldb	4,x
	adcb	#0
	stb	4,x
	ldb	3,x
	adcb	#0
	stb	3,x
	ldb	2,x
	adcb	#0
	stb	2,x
	ldb	1,x
	adcb	#0
	stb	1,x
	ldb	,x
	adcb	#0
	stb	,x

	rts
	.size __muldi3, . - __muldi3
