;;; __mulhi3 — 16-bit unsigned multiply (truncated to 16 bits)
;;; Input:  X = a (first arg), 2,s = b (second arg, after return addr)
;;; Output: X = a * b (low 16 bits)
;;; Clobbers: D
;;;
;;; Algorithm: schoolbook long multiplication using 8-bit MUL.
;;;   a = ah:al (big-endian in X)
;;;   b = bh:bl (big-endian at 2,s)
;;;   result_lo = lo(al * bl)
;;;   result_hi = hi(al * bl) + lo(ah * bl) + lo(al * bh)
;;;   (ah*bh contributes only to bits 16+, so ignored for truncated result)

	.globl	__mulhi3
	.type	__mulhi3,@function
__mulhi3:
	pshs	x		; save a; S: [ah][al][retH][retL][bh][bl]
	;       0    1     2      3     4    5

	;; Partial product 1: al * bl → full 16-bit result
	lda	1,s		; al
	ldb	5,s		; bl
	mul			; D = al * bl
	pshs	d		; save [hi(al*bl), lo(al*bl)]
	; S: [carry][res_lo][ah][al][retH][retL][bh][bl]
	;      0       1      2    3    4      5     6    7

	;; Partial product 2: ah * bl → add low byte to carry
	lda	2,s		; ah
	ldb	7,s		; bl
	mul			; D = ah * bl
	addb	,s		; carry += lo(ah * bl)
	stb	,s		; save updated carry

	;; Partial product 3: al * bh → add low byte to carry
	lda	3,s		; al
	ldb	6,s		; bh
	mul			; D = al * bh
	addb	,s		; carry += lo(al * bh)

	;; Compose result: D = result_hi:result_lo
	tfr	b,a		; A = result_hi
	ldb	1,s		; B = result_lo
	tfr	d,x		; X = result

	leas	4,s		; clean up (pshs d + pshs x)
	rts
