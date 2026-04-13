;;; divsi.inc — 32-bit signed/unsigned division and remainder
;;;
;;; ABI (gcc6809 sret-style, ABI #4):
;;;   IX        = sret pointer (caller-allocated 4-byte buffer)
;;;   2,s..3,s  = a_hi (dividend high half)
;;;   4,s..5,s  = a_lo (dividend low half)
;;;   6,s..7,s  = b_hi (divisor high half)
;;;   8,s..9,s  = b_lo (divisor low half)
;;; Result is written through IX (hi at IX+0, lo at IX+2; big-endian).
;;; Clobbers: D. Preserves: Y, U.
;;;
;;; Algorithm: restoring division (shift-and-subtract), 32 iterations.
;;; Workspace holds quotient + remainder + counter; the result (quotient
;;; for div, remainder for mod) is copied to [IX..IX+3] before return.

;;; ============================================================
;;; __udivsi3 — unsigned 32-bit division (returns quotient)
;;; ============================================================
	.globl	__udivsi3
	.type	__udivsi3,@function
__udivsi3:
	pshs	y		; preserve callee-saved Y (workspace alignment)
	leas	-9,s		; workspace: quot(4) + rem(4) + counter(1)
	; Frame layout (relative to S):
	;   S+0..3   = quotient workspace
	;   S+4..7   = remainder workspace
	;   S+8      = counter
	;   S+9..10  = saved Y
	;   S+11..12 = return addr
	;   S+13..14 = a_hi
	;   S+15..16 = a_lo
	;   S+17..18 = b_hi
	;   S+19..20 = b_lo

	; Init quotient = a (dividend)
	ldd	13,s
	std	,s
	ldd	15,s
	std	2,s

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
	cmpd	17,s		; vs divisor hi
	bhi	.Lud32_sub
	blo	.Lud32_skip
	ldd	6,s		; remainder lo
	cmpd	19,s		; vs divisor lo
	blo	.Lud32_skip

.Lud32_sub:
	ldd	6,s		; remainder lo
	subd	19,s		; - divisor lo
	std	6,s
	lda	5,s
	sbca	18,s
	sta	5,s
	lda	4,s
	sbca	17,s
	sta	4,s

	; Set quotient bit 0
	lda	3,s
	ora	#1
	sta	3,s

.Lud32_skip:
	dec	8,s
	bne	.Lud32_loop

	; Copy quotient to caller's sret buffer
	ldd	,s
	std	,x
	ldd	2,s
	std	2,x

	leas	9,s
	puls	y
	rts

;;; ============================================================
;;; __umodsi3 — unsigned 32-bit remainder
;;; ============================================================
	.globl	__umodsi3
	.type	__umodsi3,@function
__umodsi3:
	pshs	y
	leas	-9,s

	ldd	13,s
	std	,s
	ldd	15,s
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
	cmpd	17,s
	bhi	.Lum32_sub
	blo	.Lum32_skip
	ldd	6,s
	cmpd	19,s
	blo	.Lum32_skip

.Lum32_sub:
	ldd	6,s
	subd	19,s
	std	6,s
	lda	5,s
	sbca	18,s
	sta	5,s
	lda	4,s
	sbca	17,s
	sta	4,s

	lda	3,s
	ora	#1
	sta	3,s

.Lum32_skip:
	dec	8,s
	bne	.Lum32_loop

	; Copy remainder to caller's sret buffer
	ldd	4,s
	std	,x
	ldd	6,s
	std	2,x

	leas	9,s
	puls	y
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
	; Frame layout (relative to S):
	;   S+0..3   = quotient
	;   S+4..7   = remainder
	;   S+8      = counter
	;   S+9      = sign flag
	;   S+10..11 = saved Y
	;   S+12..13 = return addr
	;   S+14..15 = a_hi
	;   S+16..17 = a_lo
	;   S+18..19 = b_hi
	;   S+20..21 = b_lo

	; Determine sign: XOR high bytes of a_hi and b_hi
	lda	14,s		; a_hi high byte
	eora	18,s		; XOR b_hi high byte
	sta	9,s		; save sign flag

	; Copy dividend to quotient workspace
	ldd	14,s
	std	,s
	ldd	16,s
	std	2,s

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

	; Make b (divisor) positive if needed (in place on stack args)
	tst	18,s		; test MSB of b_hi
	bpl	.Lsd32_b_pos
	ldd	20,s		; b_lo
	coma
	comb
	addd	#1
	std	20,s
	ldd	18,s		; b_hi
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	18,s
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
	cmpd	18,s
	bhi	.Lsd32_sub
	blo	.Lsd32_skip
	ldd	6,s
	cmpd	20,s
	blo	.Lsd32_skip

.Lsd32_sub:
	ldd	6,s
	subd	20,s
	std	6,s
	lda	5,s
	sbca	19,s
	sta	5,s
	lda	4,s
	sbca	18,s
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

	; Copy quotient to caller's sret buffer
	ldd	,s
	std	,x
	ldd	2,s
	std	2,x

	leas	10,s
	puls	y
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
	; Frame layout (relative to S):
	;   S+0..3   = quotient
	;   S+4..7   = remainder
	;   S+8      = counter
	;   S+9      = sign flag (sign of a)
	;   S+10..11 = saved Y
	;   S+12..13 = return addr
	;   S+14..15 = a_hi
	;   S+16..17 = a_lo
	;   S+18..19 = b_hi
	;   S+20..21 = b_lo

	; Save sign of a (remainder follows dividend sign)
	lda	14,s		; a_hi high byte
	sta	9,s

	; Copy dividend to quotient workspace
	ldd	14,s
	std	,s
	ldd	16,s
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
	tst	18,s
	bpl	.Lsm32_b_pos
	ldd	20,s
	coma
	comb
	addd	#1
	std	20,s
	ldd	18,s
	eora	#0xFF
	eorb	#0xFF
	adcb	#0
	adca	#0
	std	18,s
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
	cmpd	18,s
	bhi	.Lsm32_sub
	blo	.Lsm32_skip
	ldd	6,s
	cmpd	20,s
	blo	.Lsm32_skip

.Lsm32_sub:
	ldd	6,s
	subd	20,s
	std	6,s
	lda	5,s
	sbca	19,s
	sta	5,s
	lda	4,s
	sbca	18,s
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

	; Copy remainder to caller's sret buffer
	ldd	4,s
	std	,x
	ldd	6,s
	std	2,x

	leas	10,s
	puls	y
	rts
