;;; divhi.inc — 16-bit signed/unsigned division and remainder
;;;
;;; All routines: X = first arg, 2,s = second arg (after return addr)
;;; Returns: X = result.  Clobbers: D.  Preserves: Y, U.
;;;
;;; Algorithm: restoring division (shift-and-subtract), 16 iterations.
;;; D = dividend → quotient (shift register), Y = remainder,
;;; X = loop counter, divisor at 4,s (after pshs y).

;;; ============================================================
;;; __udivhi3 — unsigned 16-bit division (returns quotient)
;;; ============================================================
	.globl	__udivhi3
	.type	__udivhi3,@function
__udivhi3:
	pshs	y		; save callee-saved Y
	; stack: [Y:2][ret:2][divisor:2]  offsets 0,2,4
	tfr	x,d		; D = dividend (becomes quotient)
	ldy	#0		; Y = remainder
	ldx	#16		; loop counter
.Lud_loop:
	aslb			; shift quotient left
	rola
	exg	d,y		; D = remainder, Y = partial quotient
	rolb			; shift carry (from dividend MSB) into remainder
	rola
	cmpd	4,s		; remainder >= divisor?
	blo	.Lud_skip
	subd	4,s		; remainder -= divisor
	exg	d,y		; swap back: D = quotient, Y = remainder
	orb	#1		; set quotient bit
	bra	.Lud_next
.Lud_skip:
	exg	d,y		; swap back (quotient bit stays 0)
.Lud_next:
	leax	-1,x
	bne	.Lud_loop
	tfr	d,x		; X = quotient
	puls	y
	rts

;;; ============================================================
;;; __umodhi3 — unsigned 16-bit remainder
;;; ============================================================
	.globl	__umodhi3
	.type	__umodhi3,@function
__umodhi3:
	pshs	y
	tfr	x,d
	ldy	#0
	ldx	#16
.Lum_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	4,s
	blo	.Lum_skip
	subd	4,s
	exg	d,y
	orb	#1
	bra	.Lum_next
.Lum_skip:
	exg	d,y
.Lum_next:
	leax	-1,x
	bne	.Lum_loop
	tfr	y,x		; X = remainder (not quotient)
	puls	y
	rts

;;; ============================================================
;;; __divhi3 — signed 16-bit division (returns quotient)
;;; Result sign = sign(a) XOR sign(b).
;;; ============================================================
	.globl	__divhi3
	.type	__divhi3,@function
__divhi3:
	pshs	y
	; [Y:2][ret:2][b:2]  offsets 0,2,4
	; Determine result sign: XOR high bytes of a and b
	tfr	x,d		; D = a
	eora	4,s		; A.7 = sign(a) XOR sign(b)
	pshs	a		; save sign flag
	; [sign:1][Y:2][ret:2][b:2]  offsets 0,1,3,5
	; Make a positive
	tfr	x,d		; D = a (tfr does NOT set flags)
	tsta			; test high byte for sign
	bpl	.Lsd_a_pos
	coma			; negate D (two's complement)
	comb
	addd	#1
.Lsd_a_pos:
	pshs	d		; save |a|
	; [|a|:2][sign:1][Y:2][ret:2][b:2]  offsets 0,2,3,5,7
	; Make b positive
	ldd	7,s		; load b
	bpl	.Lsd_b_pos
	coma
	comb
	addd	#1
	std	7,s		; store |b| back
.Lsd_b_pos:
	; Unsigned division: |a| / |b|
	puls	d		; D = |a|
	; [sign:1][Y:2][ret:2][|b|:2]  offsets 0,1,3,5
	ldy	#0
	ldx	#16
.Lsd_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	5,s		; compare with |b|
	blo	.Lsd_skip
	subd	5,s
	exg	d,y
	orb	#1
	bra	.Lsd_next
.Lsd_skip:
	exg	d,y
.Lsd_next:
	leax	-1,x
	bne	.Lsd_loop
	; D = quotient — fix sign if needed
	tst	,s		; check sign flag (bit 7)
	bpl	.Lsd_pos
	coma			; negate quotient
	comb
	addd	#1
.Lsd_pos:
	tfr	d,x		; X = signed quotient
	leas	1,s		; drop sign byte
	puls	y
	rts

;;; ============================================================
;;; __modhi3 — signed 16-bit remainder
;;; Remainder sign = sign of dividend (a).
;;; ============================================================
	.globl	__modhi3
	.type	__modhi3,@function
__modhi3:
	pshs	y
	; [Y:2][ret:2][b:2]
	; Save sign of a (remainder follows dividend sign)
	tfr	x,d
	pshs	a		; save sign of a
	; [asign:1][Y:2][ret:2][b:2]  offsets 0,1,3,5
	; Make a positive
	tfr	x,d		; D = a (tfr does NOT set flags)
	tsta			; test high byte for sign
	bpl	.Lsm_a_pos
	coma
	comb
	addd	#1
.Lsm_a_pos:
	pshs	d		; save |a|
	; [|a|:2][asign:1][Y:2][ret:2][b:2]  offsets 0,2,3,5,7
	; Make b positive
	ldd	7,s
	bpl	.Lsm_b_pos
	coma
	comb
	addd	#1
	std	7,s
.Lsm_b_pos:
	; Unsigned division to get remainder
	puls	d		; D = |a|
	; [asign:1][Y:2][ret:2][|b|:2]  offsets 0,1,3,5
	ldy	#0
	ldx	#16
.Lsm_loop:
	aslb
	rola
	exg	d,y
	rolb
	rola
	cmpd	5,s
	blo	.Lsm_skip
	subd	5,s
	exg	d,y
	orb	#1
	bra	.Lsm_next
.Lsm_skip:
	exg	d,y
.Lsm_next:
	leax	-1,x
	bne	.Lsm_loop
	; Y = remainder — fix sign
	tfr	y,d
	tst	,s		; check sign of original a
	bpl	.Lsm_pos
	coma
	comb
	addd	#1
.Lsm_pos:
	tfr	d,x		; X = signed remainder
	leas	1,s		; drop sign byte
	puls	y
	rts
