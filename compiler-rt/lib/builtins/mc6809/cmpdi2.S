// __cmpdi2 — 64-bit signed compare for MC6809
//
// si_int __cmpdi2(di_int a, di_int b)
//
// Returns: 0 if a <  b (signed)
//          1 if a == b
//          2 if a >  b (signed)
//
// Hand asm for the same reason as cmpsi2.S (avoid compile-time
// recursion into the libcall from within the libcall's own body).
//
// Calling convention: i32 return via sret in IX; two i64 args on the
// stack (each split into 4 i16 words, most-significant first):
//   SP+0..1:   return address
//   SP+2..3:   a word 3 (signed — most-significant)
//   SP+4..5:   a word 2 (unsigned)
//   SP+6..7:   a word 1 (unsigned)
//   SP+8..9:   a word 0 (unsigned — least-significant)
//   SP+10..11: b word 3 (signed)
//   SP+12..13: b word 2 (unsigned)
//   SP+14..15: b word 1 (unsigned)
//   SP+16..17: b word 0 (unsigned)
//   IX:        pointer to 4-byte return buffer

	.section .text,"ax",@progbits
	.globl __cmpdi2
	.type  __cmpdi2,@function
__cmpdi2:
	ldd	2,s
	cmpd	10,s
	lblt	.Lcmpdi2_less		// signed compare on top word
	lbgt	.Lcmpdi2_greater
	ldd	4,s
	cmpd	12,s
	lblo	.Lcmpdi2_less		// unsigned from here down
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
