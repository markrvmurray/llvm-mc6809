;;; cmpsi2.inc — 32-bit signed compare runtime entry point.
;;;
;;; Mirrors compiler-rt/lib/builtins/mc6809/cmpsi2.S so the
;;; execution-test mc6809rt.s can `.include` it without depending
;;; on a compiler-rt build. Update both files in sync.
;;;
;;; si_int __cmpsi2(si_int a, si_int b)
;;;   Returns: 0 if a <  b (signed)
;;;            1 if a == b
;;;            2 if a >  b (signed)
;;;
;;; Calling convention (i32 return via sret in IX, all i32 args on
;;; stack):
;;;   SP+0..1:  return address
;;;   SP+2..3:  a_hi (signed i16 — most-significant half of a)
;;;   SP+4..5:  a_lo (unsigned i16 — least-significant half)
;;;   SP+6..7:  b_hi
;;;   SP+8..9:  b_lo
;;;   IX:       pointer to 4-byte return buffer (hi at IX+0, lo at IX+2)

	.globl __cmpsi2
	.type  __cmpsi2,@function
__cmpsi2:
	ldd	2,s
	cmpd	6,s
	blt	.Lcmpsi2_less		; signed: a_hi < b_hi  → a < b
	bgt	.Lcmpsi2_greater	; signed: a_hi > b_hi  → a > b
	ldd	4,s
	cmpd	8,s
	blo	.Lcmpsi2_less		; unsigned: a_lo < b_lo
	bhi	.Lcmpsi2_greater
	ldd	#1			; a == b
	bra	.Lcmpsi2_store
.Lcmpsi2_less:
	ldd	#0
	bra	.Lcmpsi2_store
.Lcmpsi2_greater:
	ldd	#2
.Lcmpsi2_store:
	std	2,x			; low half of i32 return = result
	ldd	#0
	std	,x			; high half = 0
	rts
	.size __cmpsi2, . - __cmpsi2
