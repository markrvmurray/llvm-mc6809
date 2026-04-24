// __cmpsi2 — 32-bit signed compare for MC6809
//
// si_int __cmpsi2(si_int a, si_int b)
//
// Returns: 0 if a <  b (signed)
//          1 if a == b
//          2 if a >  b (signed)
//
// Why hand asm: the C version (compiled with the MC6809 backend that
// libcalls i32 compare to __cmpsi2) infinite-recurses when any mid-
// end pass introduces an i32 compare in its own body. Hand asm breaks
// the recursion by construction — the compiler never sees the body.
//
// Calling convention (i32 return via sret in IX, all i32 args on stack):
//   SP+0..1:  return address
//   SP+2..3:  a_hi (signed i16 — most-significant half of a)
//   SP+4..5:  a_lo (unsigned i16 — least-significant half)
//   SP+6..7:  b_hi
//   SP+8..9:  b_lo
//   IX:       pointer to 4-byte return buffer (hi at IX+0, lo at IX+2)
//
// Result value fits in the low 16 bits, so the high half is always 0.

	.section .text,"ax",@progbits
	.globl __cmpsi2
	.type  __cmpsi2,@function
__cmpsi2:
	ldd	2,s
	cmpd	6,s
	blt	.Lcmpsi2_less		// signed: a_hi < b_hi  → a < b
	bgt	.Lcmpsi2_greater	// signed: a_hi > b_hi  → a > b
	ldd	4,s
	cmpd	8,s
	blo	.Lcmpsi2_less		// unsigned: a_lo < b_lo
	bhi	.Lcmpsi2_greater
	ldd	#1			// a == b
	bra	.Lcmpsi2_store
.Lcmpsi2_less:
	ldd	#0
	bra	.Lcmpsi2_store
.Lcmpsi2_greater:
	ldd	#2
.Lcmpsi2_store:
	std	2,x			// low half of i32 return = result
	ldd	#0
	std	,x			// high half = 0
	rts
	.size __cmpsi2, . - __cmpsi2
