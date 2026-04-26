;;; ucmpsi2.inc — 32-bit unsigned compare runtime entry point.
;;;
;;; Mirrors compiler-rt/lib/builtins/mc6809/ucmpsi2.S so the
;;; execution-test mc6809rt.s can `.include` it without depending
;;; on a compiler-rt build. Update both files in sync.
;;;
;;; si_int __ucmpsi2(su_int a, su_int b)
;;;   Returns: 0 if a <  b (unsigned)
;;;            1 if a == b
;;;            2 if a >  b (unsigned)
;;;
;;; ABI / stack layout: see cmpsi2.inc header. Same as cmpsi2 but
;;; both halves of the 32-bit operands are unsigned-compared.

	.globl __ucmpsi2
	.type  __ucmpsi2,@function
__ucmpsi2:
	ldd	2,s
	cmpd	6,s
	blo	.Lucmpsi2_less		; unsigned: a_hi < b_hi
	bhi	.Lucmpsi2_greater
	ldd	4,s
	cmpd	8,s
	blo	.Lucmpsi2_less
	bhi	.Lucmpsi2_greater
	ldd	#1			; a == b
	bra	.Lucmpsi2_store
.Lucmpsi2_less:
	ldd	#0
	bra	.Lucmpsi2_store
.Lucmpsi2_greater:
	ldd	#2
.Lucmpsi2_store:
	std	2,x
	ldd	#0
	std	,x
	rts
	.size __ucmpsi2, . - __ucmpsi2
