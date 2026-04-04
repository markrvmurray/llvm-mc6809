;;; divsi.inc — 32-bit signed/unsigned division and remainder
;;;
;;; Convention (same as __mulsi3):
;;;   Input:  X = a_hi, 2,s = a_lo, 4,s = b_hi, 6,s = b_lo
;;;   Output: X = result_lo, result_hi written to 2,s
;;;   Clobbers: D.  Preserves: Y, U.
;;;
;;; Algorithm: restoring division (shift-and-subtract), 32 iterations.

;;; ============================================================
;;; __udivsi3 — unsigned 32-bit division (returns quotient)
;;; ============================================================
	.globl	__udivsi3
	.type	__udivsi3,@function
__udivsi3:
	pshs	y
	leas	-9,s		; workspace: quot(4) + rem(4) + counter(1)
	; S+0..3  = quotient (= dividend initially)
	; S+4..7  = remainder (= 0 initially)
	; S+8     = counter
	; S+9,10  = saved Y
	; S+11,12 = return addr
	; S+13,14 = a_lo
	; S+15,16 = b_hi (divisor)
	; S+17,18 = b_lo

	; Init quotient = dividend (a_hi:a_lo)
	stx	,s		; quotient hi = a_hi
	ldd	13,s
	std	2,s		; quotient lo = a_lo

	; Init remainder = 0
	ldd	#0
	std	4,s
	std	6,s

	; Counter = 32
	lda	#32
	sta	8,s

.Lud32_loop:
	; Shift quotient left (32-bit)
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s		; MSB → carry

	; Rotate carry into remainder (32-bit)
	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	; Compare remainder with divisor (32-bit unsigned)
	ldd	4,s		; remainder hi
	cmpd	15,s		; vs divisor hi
	bhi	.Lud32_sub
	blo	.Lud32_skip
	ldd	6,s		; remainder lo
	cmpd	17,s		; vs divisor lo
	blo	.Lud32_skip

.Lud32_sub:
	; remainder -= divisor (32-bit)
	ldd	6,s		; remainder lo
	subd	17,s		; - divisor lo
	std	6,s
	lda	5,s
	sbca	16,s
	sta	5,s
	lda	4,s
	sbca	15,s
	sta	4,s

	; Set quotient bit 0
	lda	3,s
	ora	#1
	sta	3,s

.Lud32_skip:
	dec	8,s
	bne	.Lud32_loop

	; Return quotient: X = lo, 2,s = hi
	ldx	2,s		; result lo
	ldd	,s		; result hi
	leas	9,s
	puls	y
	std	2,s		; write hi to caller slot
	rts

;;; ============================================================
;;; __umodsi3 — unsigned 32-bit remainder
;;; ============================================================
	.globl	__umodsi3
	.type	__umodsi3,@function
__umodsi3:
	pshs	y
	leas	-9,s

	stx	,s
	ldd	13,s
	std	2,s

	ldd	#0
	std	4,s
	std	6,s

	lda	#32
	sta	8,s

.Lum32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	15,s
	bhi	.Lum32_sub
	blo	.Lum32_skip
	ldd	6,s
	cmpd	17,s
	blo	.Lum32_skip

.Lum32_sub:
	ldd	6,s
	subd	17,s
	std	6,s
	lda	5,s
	sbca	16,s
	sta	5,s
	lda	4,s
	sbca	15,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lum32_skip:
	dec	8,s
	bne	.Lum32_loop

	; Return remainder: X = lo, 2,s = hi
	ldx	6,s		; remainder lo
	ldd	4,s		; remainder hi
	leas	9,s
	puls	y
	std	2,s
	rts

;;; ============================================================
;;; __divsi3 — signed 32-bit division (returns quotient)
;;; Result sign = sign(a) XOR sign(b).
;;; ============================================================
	.globl	__divsi3
	.type	__divsi3,@function
__divsi3:
	pshs	y
	leas	-10,s		; quot(4) + rem(4) + counter(1) + sign(1)
	; S+0..3  = quotient
	; S+4..7  = remainder
	; S+8     = counter
	; S+9     = sign flag
	; S+10,11 = saved Y
	; S+12,13 = return addr
	; S+14,15 = a_lo
	; S+16,17 = b_hi
	; S+18,19 = b_lo

	; Determine sign: XOR high bytes of a and b
	tfr	x,d		; A = a_hi high byte
	eora	16,s		; XOR with b_hi high byte
	sta	9,s		; save sign flag

	; Copy dividend to quotient workspace
	stx	,s		; quotient hi = a_hi
	ldd	14,s
	std	2,s		; quotient lo = a_lo

	; Make a (quotient) positive if needed
	tst	,s		; test MSB of a_hi
	bpl	.Lsd32_a_pos
	; Negate quotient (32-bit): complement + 1
	; NOTE: use EOR not COM for hi word — COM sets carry, corrupting chain
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsd32_a_pos:

	; Make b (divisor) positive if needed
	tst	16,s		; test MSB of b_hi
	bpl	.Lsd32_b_pos
	; Negate divisor in-place on stack
	ldd	18,s		; b_lo
	coma
	comb
	addd	#1
	std	18,s
	ldd	16,s		; b_hi
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	16,s
.Lsd32_b_pos:

	; Init remainder = 0
	ldd	#0
	std	4,s
	std	6,s

	; Counter = 32
	lda	#32
	sta	8,s

.Lsd32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	16,s		; divisor hi (may have been negated)
	bhi	.Lsd32_sub
	blo	.Lsd32_skip
	ldd	6,s
	cmpd	18,s		; divisor lo
	blo	.Lsd32_skip

.Lsd32_sub:
	ldd	6,s
	subd	18,s
	std	6,s
	lda	5,s
	sbca	17,s
	sta	5,s
	lda	4,s
	sbca	16,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lsd32_skip:
	dec	8,s
	bne	.Lsd32_loop

	; Negate quotient if sign flag says so
	tst	9,s
	bpl	.Lsd32_pos
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsd32_pos:

	ldx	2,s		; result lo
	ldd	,s		; result hi
	leas	10,s
	puls	y
	std	2,s
	rts

;;; ============================================================
;;; __modsi3 — signed 32-bit remainder
;;; Remainder sign = sign of dividend (a).
;;; ============================================================
	.globl	__modsi3
	.type	__modsi3,@function
__modsi3:
	pshs	y
	leas	-10,s
	; S+0..3  = quotient
	; S+4..7  = remainder
	; S+8     = counter
	; S+9     = sign flag (sign of a)
	; S+10,11 = saved Y
	; S+12,13 = return addr
	; S+14,15 = a_lo
	; S+16,17 = b_hi
	; S+18,19 = b_lo

	; Save sign of a (remainder follows dividend sign)
	tfr	x,d
	sta	9,s		; A = high byte of a_hi

	; Copy dividend to quotient workspace
	stx	,s
	ldd	14,s
	std	2,s

	; Make a positive if needed
	tst	,s
	bpl	.Lsm32_a_pos
	ldd	2,s
	coma
	comb
	addd	#1
	std	2,s
	ldd	,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	,s
.Lsm32_a_pos:

	; Make b positive if needed
	tst	16,s
	bpl	.Lsm32_b_pos
	ldd	18,s
	coma
	comb
	addd	#1
	std	18,s
	ldd	16,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	16,s
.Lsm32_b_pos:

	ldd	#0
	std	4,s
	std	6,s

	lda	#32
	sta	8,s

.Lsm32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s

	rol	7,s
	rol	6,s
	rol	5,s
	rol	4,s

	ldd	4,s
	cmpd	16,s
	bhi	.Lsm32_sub
	blo	.Lsm32_skip
	ldd	6,s
	cmpd	18,s
	blo	.Lsm32_skip

.Lsm32_sub:
	ldd	6,s
	subd	18,s
	std	6,s
	lda	5,s
	sbca	17,s
	sta	5,s
	lda	4,s
	sbca	16,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lsm32_skip:
	dec	8,s
	bne	.Lsm32_loop

	; Negate remainder if a was negative
	tst	9,s
	bpl	.Lsm32_pos
	ldd	6,s
	coma
	comb
	addd	#1
	std	6,s
	ldd	4,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	4,s
.Lsm32_pos:

	ldx	6,s		; remainder lo
	ldd	4,s		; remainder hi
	leas	10,s
	puls	y
	std	2,s
	rts
