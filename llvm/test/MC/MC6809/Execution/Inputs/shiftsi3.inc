;;; shiftsi3.inc — 32-bit variable shift library functions
;;;
;;; ABI: X = a_hi, 2,s = a_lo, 4,s = b_hi, 6,s = b_lo
;;; (shift amount = b_lo, range 0-31)
;;; Returns: X = result_lo, result_hi written to 2,s.
;;; Clobbers: D.  Preserves: Y, U.
;;;
;;; All three functions arrange the 32-bit value in a contiguous 4-byte
;;; workspace on the stack and shift all 4 bytes through carry.

;;; __ashlsi3 — 32-bit left shift
	.globl	__ashlsi3
	.type	__ashlsi3,@function
__ashlsi3:
	pshs	y
	leas	-4,s		; workspace for 32-bit value
	; S+0..3=workspace  S+4,5=Y  S+6,7=ret  S+8,9=a_lo  S+10,11=b_hi  S+12,13=b_lo
	stx	,s		; workspace[0,1] = a_hi (MSB)
	ldd	8,s
	std	2,s		; workspace[2,3] = a_lo (LSB)
	lda	13,s		; shift count
	beq	.Lshl32_done
.Lshl32_loop:
	asl	3,s		; shift byte 3 (LSB) left
	rol	2,s
	rol	1,s
	rol	,s		; shift byte 0 (MSB) left
	deca
	bne	.Lshl32_loop
.Lshl32_done:
	ldx	2,s		; X = result_lo
	ldd	,s		; D = result_hi
	leas	4,s		; drop workspace
	puls	y
	std	2,s		; write result_hi to return slot
	rts

;;; __lshrsi3 — 32-bit logical right shift
	.globl	__lshrsi3
	.type	__lshrsi3,@function
__lshrsi3:
	pshs	y
	leas	-4,s
	stx	,s
	ldd	8,s
	std	2,s
	lda	13,s
	beq	.Llsr32_done
.Llsr32_loop:
	lsr	,s		; shift byte 0 (MSB) right
	ror	1,s
	ror	2,s
	ror	3,s		; shift byte 3 (LSB) right
	deca
	bne	.Llsr32_loop
.Llsr32_done:
	ldx	2,s
	ldd	,s
	leas	4,s
	puls	y
	std	2,s
	rts

;;; __ashrsi3 — 32-bit arithmetic right shift
	.globl	__ashrsi3
	.type	__ashrsi3,@function
__ashrsi3:
	pshs	y
	leas	-4,s
	stx	,s
	ldd	8,s
	std	2,s
	lda	13,s
	beq	.Lasr32_done
.Lasr32_loop:
	asr	,s		; arithmetic shift byte 0 (MSB): sign preserved
	ror	1,s
	ror	2,s
	ror	3,s
	deca
	bne	.Lasr32_loop
.Lasr32_done:
	ldx	2,s
	ldd	,s
	leas	4,s
	puls	y
	std	2,s
	rts
