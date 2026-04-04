;;; shiftsi3.inc — 32-bit variable shift library functions
;;;
;;; ABI (codegen convention): X = a_hi, 2,s = a_lo, B = shift count (0-31)
;;; Returns: X = result_hi, result_lo written to 2,s.
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
	; S+0..3=workspace  S+4,5=Y  S+6,7=ret  S+8,9=a_lo
	stx	,s		; workspace[0,1] = a_hi (MSB)
	lda	8,s
	sta	2,s
	lda	9,s
	sta	3,s		; workspace[2,3] = a_lo (LSB)
	tstb			; shift count in B
	beq	.Lshl32_done
.Lshl32_loop:
	asl	3,s		; shift byte 3 (LSB) left
	rol	2,s
	rol	1,s
	rol	,s		; shift byte 0 (MSB) left
	decb
	bne	.Lshl32_loop
.Lshl32_done:
	ldx	,s		; X = result_hi
	ldd	2,s		; D = result_lo
	leas	4,s		; drop workspace
	puls	y
	std	2,s		; write result_lo to return slot
	rts

;;; __lshrsi3 — 32-bit logical right shift
	.globl	__lshrsi3
	.type	__lshrsi3,@function
__lshrsi3:
	pshs	y
	leas	-4,s
	stx	,s
	lda	8,s
	sta	2,s
	lda	9,s
	sta	3,s
	tstb
	beq	.Llsr32_done
.Llsr32_loop:
	lsr	,s		; shift byte 0 (MSB) right
	ror	1,s
	ror	2,s
	ror	3,s		; shift byte 3 (LSB) right
	decb
	bne	.Llsr32_loop
.Llsr32_done:
	ldx	,s		; X = result_hi
	ldd	2,s		; D = result_lo
	leas	4,s
	puls	y
	std	2,s		; write result_lo to return slot
	rts

;;; __ashrsi3 — 32-bit arithmetic right shift
	.globl	__ashrsi3
	.type	__ashrsi3,@function
__ashrsi3:
	pshs	y
	leas	-4,s
	stx	,s
	lda	8,s
	sta	2,s
	lda	9,s
	sta	3,s
	tstb
	beq	.Lasr32_done
.Lasr32_loop:
	asr	,s		; arithmetic shift byte 0 (MSB): sign preserved
	ror	1,s
	ror	2,s
	ror	3,s
	decb
	bne	.Lasr32_loop
.Lasr32_done:
	ldx	,s		; X = result_hi
	ldd	2,s		; D = result_lo
	leas	4,s
	puls	y
	std	2,s		; write result_lo to return slot
	rts
