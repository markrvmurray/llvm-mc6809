// __ucmpdi2 — 64-bit unsigned compare for MC6809
//
// si_int __ucmpdi2(du_int a, du_int b)
//
// Returns: 0 if a <  b (unsigned)
//          1 if a == b
//          2 if a >  b (unsigned)
//
// Hand asm (see cmpsi2.S for rationale). Same stack/register ABI as
// cmpdi2.S but every word comparison is unsigned.

	.section .text,"ax",@progbits
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
