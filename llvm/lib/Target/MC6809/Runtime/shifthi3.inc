;;; shifthi3.inc — 16-bit variable shift library functions
;;;
;;; Calling convention (matches LLVM MC6809 codegen):
;;;   X = value to shift, B = shift amount (unsigned byte).
;;; Returns: X = result.  Clobbers: D.  Preserves: Y, U.

;;; __ashlhi3 — 16-bit left shift
	.globl	__ashlhi3
	.type	__ashlhi3,@function
__ashlhi3:
	tstb
	beq	.Lshl16_zero
	pshs	b		; push count to stack
	tfr	x,d		; D = value (A=hi, B=lo)
.Lshl16_loop:
	aslb			; shift lo left, carry → C
	rola			; shift hi left, C → bit 0
	dec	,s		; decrement count
	bne	.Lshl16_loop
	leas	1,s		; drop counter
	tfr	d,x
	rts
.Lshl16_zero:
	rts			; X already has the value

;;; __lshrhi3 — 16-bit logical right shift
	.globl	__lshrhi3
	.type	__lshrhi3,@function
__lshrhi3:
	tstb
	beq	.Llsr16_zero
	pshs	b
	tfr	x,d
.Llsr16_loop:
	lsra			; shift hi right, bit 0 → C
	rorb			; shift lo right, C → bit 7
	dec	,s
	bne	.Llsr16_loop
	leas	1,s
	tfr	d,x
	rts
.Llsr16_zero:
	rts

;;; __ashrhi3 — 16-bit arithmetic right shift
	.globl	__ashrhi3
	.type	__ashrhi3,@function
__ashrhi3:
	tstb
	beq	.Lasr16_zero
	pshs	b
	tfr	x,d
.Lasr16_loop:
	asra			; arithmetic shift hi, sign preserved, bit 0 → C
	rorb			; shift lo right, C → bit 7
	dec	,s
	bne	.Lasr16_loop
	leas	1,s
	tfr	d,x
	rts
.Lasr16_zero:
	rts
