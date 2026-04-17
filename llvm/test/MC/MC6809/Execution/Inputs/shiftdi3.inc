;; shiftdi3.inc — 64-bit shift runtime for MC6809
;;
;; __ashrdi3(di_int a, int b) — arithmetic shift right
;;
;; Calling convention (sret for i64 return):
;;   X  = pointer to 8-byte result buffer
;;   Stack: S+2..S+9 = i64 input, S+10..S+11 = i16 shift count

	.globl	__ashrdi3
	.type	__ashrdi3,@function
__ashrdi3:
	;; Copy 8-byte input from stack to result buffer at X.
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
	;; Load shift count from stack (low byte, masked to 0-63).
	ldb	11,s
	andb	#63
	beq	.Lashrdi3_done
.Lashrdi3_loop:
	asr	,x
	ror	1,x
	ror	2,x
	ror	3,x
	ror	4,x
	ror	5,x
	ror	6,x
	ror	7,x
	decb
	bne	.Lashrdi3_loop
.Lashrdi3_done:
	rts
	.size	__ashrdi3, . - __ashrdi3
