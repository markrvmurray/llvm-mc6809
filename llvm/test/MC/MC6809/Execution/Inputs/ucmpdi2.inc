;;; ucmpdi2.inc — 64-bit unsigned compare runtime entry point.
;;;
;;; Mirrors compiler-rt/lib/builtins/mc6809/ucmpdi2.S so the
;;; execution-test mc6809rt.s can `.include` it without depending
;;; on a compiler-rt build. Update both files in sync.
;;;
;;; si_int __ucmpdi2(du_int a, du_int b)
;;;   Returns: 0 if a <  b (unsigned)
;;;            1 if a == b
;;;            2 if a >  b (unsigned)

	.globl __ucmpdi2
	.type  __ucmpdi2,@function
__ucmpdi2:
	ldd	2,s
	cmpd	10,s
	lblo	.Lucmpdi2_less
	lbhi	.Lucmpdi2_greater
	ldd	4,s
	cmpd	12,s
	lblo	.Lucmpdi2_less
	lbhi	.Lucmpdi2_greater
	ldd	6,s
	cmpd	14,s
	lblo	.Lucmpdi2_less
	lbhi	.Lucmpdi2_greater
	ldd	8,s
	cmpd	16,s
	lblo	.Lucmpdi2_less
	lbhi	.Lucmpdi2_greater
	ldd	#1
	bra	.Lucmpdi2_store
.Lucmpdi2_less:
	ldd	#0
	bra	.Lucmpdi2_store
.Lucmpdi2_greater:
	ldd	#2
.Lucmpdi2_store:
	std	2,x
	ldd	#0
	std	,x
	rts
	.size __ucmpdi2, . - __ucmpdi2
