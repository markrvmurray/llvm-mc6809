; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=100000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Branch instruction execution tests.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; Test BEQ: 5 - 5 = 0, should branch
	lda	#5
	cmpa	#5
	beq	.Leq_yes
	ldx	#msg_no
	bra	.Leq_out
.Leq_yes:
	ldx	#msg_yes
.Leq_out:
	jsr	putstr
	jsr	putnl

	;; Test BNE: 5 - 3 != 0, should branch
	lda	#5
	cmpa	#3
	bne	.Lne_yes
	ldx	#msg_no
	bra	.Lne_out
.Lne_yes:
	ldx	#msg_yes
.Lne_out:
	jsr	putstr
	jsr	putnl

	;; Test BHI: 5 > 3 (unsigned), should branch
	lda	#5
	cmpa	#3
	bhi	.Lhi_yes
	ldx	#msg_no
	bra	.Lhi_out
.Lhi_yes:
	ldx	#msg_yes
.Lhi_out:
	jsr	putstr
	jsr	putnl

	;; Test BLO: 3 < 5 (unsigned), should branch
	lda	#3
	cmpa	#5
	blo	.Llo_yes
	ldx	#msg_no
	bra	.Llo_out
.Llo_yes:
	ldx	#msg_yes
.Llo_out:
	jsr	putstr
	jsr	putnl

	rts

msg_yes:
	.asciz	"YES"
msg_no:
	.asciz	"NO"

; CHECK: YES
; CHECK-NEXT: YES
; CHECK-NEXT: YES
; CHECK-NEXT: YES
