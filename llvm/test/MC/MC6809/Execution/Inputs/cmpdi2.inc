;;; cmpdi2.inc — 64-bit signed compare runtime entry point.
;;;
;;; Mirrors compiler-rt/lib/builtins/mc6809/cmpdi2.S so the
;;; execution-test mc6809rt.s can `.include` it without depending
;;; on a compiler-rt build. Update both files in sync.
;;;
;;; si_int __cmpdi2(di_int a, di_int b)
;;;   Returns: 0 if a <  b (signed)
;;;            1 if a == b
;;;            2 if a >  b (signed)
;;;
;;; Calling convention: i32 return via sret in IX; two i64 args on
;;; the stack, each split into 4 i16 words, most-significant first.

	.globl __cmpdi2
	.type  __cmpdi2,@function
__cmpdi2:
	ldd	2,s
	cmpd	10,s
	lblt	.Lcmpdi2_less		; signed compare on top word
	lbgt	.Lcmpdi2_greater
	ldd	4,s
	cmpd	12,s
	lblo	.Lcmpdi2_less		; unsigned from here down
	lbhi	.Lcmpdi2_greater
	ldd	6,s
	cmpd	14,s
	lblo	.Lcmpdi2_less
	lbhi	.Lcmpdi2_greater
	ldd	8,s
	cmpd	16,s
	lblo	.Lcmpdi2_less
	lbhi	.Lcmpdi2_greater
	ldd	#1
	bra	.Lcmpdi2_store
.Lcmpdi2_less:
	ldd	#0
	bra	.Lcmpdi2_store
.Lcmpdi2_greater:
	ldd	#2
.Lcmpdi2_store:
	std	2,x
	ldd	#0
	std	,x
	rts
	.size __cmpdi2, . - __cmpdi2
