; A hand-written routine with no CFI, for mc6809-cfi annotate.
	.text
	.globl	work
	.type	work,@function
work:
	pshs	u,y
#ifdef __6309__
	tfr	x,d
#else
	pshs	x
	puls	d
#endif
	tstb
	beq	done
	pshs	d
	leas	2,s
	lbra	finish
done:
	puls	u,y
	rts
	.size	work, .-work
finish:
	puls	u,y
	rts
