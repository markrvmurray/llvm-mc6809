;; shiftdi3.inc — 64-bit shift runtime for MC6809
;;
;; Calling convention (sret for i64 return):
;;   X  = pointer to 8-byte result buffer
;;   Stack: S+2..S+9 = i64 input, S+10..S+11 = i16 shift count

	.globl	__ashldi3
	.type	__ashldi3,@function
__ashldi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
	ldb	11,s
	andb	#63
	beq	.Lashldi3_done
.Lashldi3_loop:
	asl	7,x
	rol	6,x
	rol	5,x
	rol	4,x
	rol	3,x
	rol	2,x
	rol	1,x
	rol	,x
	decb
	bne	.Lashldi3_loop
.Lashldi3_done:
	rts
	.size	__ashldi3, . - __ashldi3

	.globl	__ashrdi3
	.type	__ashrdi3,@function
__ashrdi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
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

	.globl	__lshrdi3
	.type	__lshrdi3,@function
__lshrdi3:
	ldd	2,s
	std	,x
	ldd	4,s
	std	2,x
	ldd	6,s
	std	4,x
	ldd	8,s
	std	6,x
	ldb	11,s
	andb	#63
	beq	.Llshrdi3_done
.Llshrdi3_loop:
	lsr	,x
	ror	1,x
	ror	2,x
	ror	3,x
	ror	4,x
	ror	5,x
	ror	6,x
	ror	7,x
	decb
	bne	.Llshrdi3_loop
.Llshrdi3_done:
	rts
	.size	__lshrdi3, . - __lshrdi3
