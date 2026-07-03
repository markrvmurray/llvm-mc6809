;;; shiftsi3.inc — 32-bit variable shift library functions
;;;
;;; ABI (matches compiler-rt lib/builtins/mc6809/*si3.S):
;;;   IX        = sret pointer (caller-allocated 4-byte buffer)
;;;   2,s..3,s  = value high half
;;;   4,s..5,s  = value low half
;;;   6,s..7,s  = shift count (i16 stack slot; low byte at 7,s)
;;;
;;; The count is read from its stack slot, NOT from B: the compiler
;;; passes all three arguments on the stack, and relying on the count
;;; happening to linger in B after the caller's marshalling store was
;;; a latent accident that broke once stores stage through D.
;;; Result is written through IX: hi at IX+0, lo at IX+2 (big-endian).
;;; Clobbers: D. Preserves: Y, U.
;;;
;;; The shift is performed in place at [IX..IX+3]. The destination
;;; buffer is the workspace, so no stack frame is needed.

;;; Copy the input value (2,s..5,s) into [IX..IX+3] using A only.
;;; Macro-style sequence used by all three shift functions.

;;; __ashlsi3 — 32-bit left shift
	.globl	__ashlsi3
	.type	__ashlsi3,@function
__ashlsi3:
	lda	2,s		; copy value_hi byte 0
	sta	,x
	lda	3,s		; copy value_hi byte 1
	sta	1,x
	lda	4,s		; copy value_lo byte 0
	sta	2,x
	lda	5,s		; copy value_lo byte 1
	sta	3,x
	ldb	7,s		; shift count (low byte of the i16 slot)
	andb	#31
	beq	.Lshl32_done
.Lshl32_loop:
	asl	3,x		; shift LSB byte left
	rol	2,x
	rol	1,x
	rol	,x		; shift MSB byte left
	decb
	bne	.Lshl32_loop
.Lshl32_done:
	rts

;;; __lshrsi3 — 32-bit logical right shift
	.globl	__lshrsi3
	.type	__lshrsi3,@function
__lshrsi3:
	lda	2,s
	sta	,x
	lda	3,s
	sta	1,x
	lda	4,s
	sta	2,x
	lda	5,s
	sta	3,x
	ldb	7,s		; shift count (low byte of the i16 slot)
	andb	#31
	beq	.Llsr32_done
.Llsr32_loop:
	lsr	,x		; shift MSB byte right (zero-fill)
	ror	1,x
	ror	2,x
	ror	3,x		; shift LSB byte right
	decb
	bne	.Llsr32_loop
.Llsr32_done:
	rts

;;; __ashrsi3 — 32-bit arithmetic right shift (sign-preserving)
	.globl	__ashrsi3
	.type	__ashrsi3,@function
__ashrsi3:
	lda	2,s
	sta	,x
	lda	3,s
	sta	1,x
	lda	4,s
	sta	2,x
	lda	5,s
	sta	3,x
	ldb	7,s		; shift count (low byte of the i16 slot)
	andb	#31
	beq	.Lasr32_done
.Lasr32_loop:
	asr	,x		; arithmetic shift MSB byte right
	ror	1,x
	ror	2,x
	ror	3,x
	decb
	bne	.Lasr32_loop
.Lasr32_done:
	rts
