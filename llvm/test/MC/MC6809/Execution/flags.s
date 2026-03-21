; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=200000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Flag behavior execution tests.
; Tests CC register flags: Z, N, C, V via conditional branches.
; Prints Y (flag set) or N (flag not set).

.include "runtime.inc"

	.section .rom,"ax",@progbits

;; Helper: print 'Y' if branch taken, 'N' if not.
;; Usage: conditional-branch to .Lyes_N, falls through to .Lno_N.
;; Each test uses a unique label suffix.

	.globl	test_main
test_main:
	;; ============ ZERO FLAG ============

	;; Z set after loading 0
	lda	#0x00
	beq	.Lz1_yes
	lda	#0x4e		; 'N'
	bra	.Lz1_out
.Lz1_yes:
	lda	#0x59		; 'Y'
.Lz1_out:
	jsr	putchar
	jsr	putnl
; CHECK: Y

	;; Z clear after loading non-zero
	lda	#0x42
	beq	.Lz2_yes
	lda	#0x4e		; 'N'
	bra	.Lz2_out
.Lz2_yes:
	lda	#0x59		; 'Y'
.Lz2_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; Z set after subtract equal values
	lda	#0x55
	suba	#0x55
	beq	.Lz3_yes
	lda	#0x4e
	bra	.Lz3_out
.Lz3_yes:
	lda	#0x59
.Lz3_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; ============ NEGATIVE FLAG ============

	;; N set after loading negative value
	lda	#0x80
	bmi	.Ln1_yes
	lda	#0x4e
	bra	.Ln1_out
.Ln1_yes:
	lda	#0x59
.Ln1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; N clear after loading positive value
	lda	#0x7f
	bmi	.Ln2_yes
	lda	#0x4e
	bra	.Ln2_out
.Ln2_yes:
	lda	#0x59
.Ln2_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; ============ CARRY FLAG ============

	;; C set after 0xFF + 0x01 (unsigned overflow)
	lda	#0xff
	adda	#0x01
	bcs	.Lc1_yes
	lda	#0x4e
	bra	.Lc1_out
.Lc1_yes:
	lda	#0x59
.Lc1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; C clear after 0x01 + 0x01
	lda	#0x01
	adda	#0x01
	bcs	.Lc2_yes
	lda	#0x4e
	bra	.Lc2_out
.Lc2_yes:
	lda	#0x59
.Lc2_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; C set after subtract borrow: 0x00 - 0x01
	lda	#0x00
	suba	#0x01
	bcs	.Lc3_yes
	lda	#0x4e
	bra	.Lc3_out
.Lc3_yes:
	lda	#0x59
.Lc3_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; C clear after subtract no borrow: 0x05 - 0x03
	lda	#0x05
	suba	#0x03
	bcs	.Lc4_yes
	lda	#0x4e
	bra	.Lc4_out
.Lc4_yes:
	lda	#0x59
.Lc4_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; ============ OVERFLOW FLAG ============

	;; V set: 0x7F + 0x01 = 0x80 (positive + positive = negative)
	lda	#0x7f
	adda	#0x01
	bvs	.Lv1_yes
	lda	#0x4e
	bra	.Lv1_out
.Lv1_yes:
	lda	#0x59
.Lv1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; V clear: 0x01 + 0x01 = 0x02 (no overflow)
	lda	#0x01
	adda	#0x01
	bvs	.Lv2_yes
	lda	#0x4e
	bra	.Lv2_out
.Lv2_yes:
	lda	#0x59
.Lv2_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; V set: 0x80 - 0x01 = 0x7F (negative - positive = positive)
	lda	#0x80
	suba	#0x01
	bvs	.Lv3_yes
	lda	#0x4e
	bra	.Lv3_out
.Lv3_yes:
	lda	#0x59
.Lv3_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; ============ COMPOUND CONDITIONS ============

	;; BGE (signed >=): 5 >= 3
	lda	#5
	cmpa	#3
	bge	.Lge1_yes
	lda	#0x4e
	bra	.Lge1_out
.Lge1_yes:
	lda	#0x59
.Lge1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; BLT (signed <): -1 < 0 (0xFF < 0x00 signed)
	lda	#0xff
	cmpa	#0x00
	blt	.Llt1_yes
	lda	#0x4e
	bra	.Llt1_out
.Llt1_yes:
	lda	#0x59
.Llt1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; BGT (signed >): 5 > 3
	lda	#5
	cmpa	#3
	bgt	.Lgt1_yes
	lda	#0x4e
	bra	.Lgt1_out
.Lgt1_yes:
	lda	#0x59
.Lgt1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; BLE (signed <=): 3 <= 3
	lda	#3
	cmpa	#3
	ble	.Lle1_yes
	lda	#0x4e
	bra	.Lle1_out
.Lle1_yes:
	lda	#0x59
.Lle1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; BLE (signed <=): 5 <= 3 → false
	lda	#5
	cmpa	#3
	ble	.Lle2_yes
	lda	#0x4e
	bra	.Lle2_out
.Lle2_yes:
	lda	#0x59
.Lle2_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	;; ============ ANDCC / ORCC ============

	;; ORCC sets carry, then test it
	andcc	#0xfe		; clear carry first
	orcc	#0x01		; set carry
	bcs	.Loc1_yes
	lda	#0x4e
	bra	.Loc1_out
.Loc1_yes:
	lda	#0x59
.Loc1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; ANDCC clears carry, then test it
	orcc	#0x01		; set carry first
	andcc	#0xfe		; clear carry
	bcs	.Lac1_yes
	lda	#0x4e
	bra	.Lac1_out
.Lac1_yes:
	lda	#0x59
.Lac1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: N

	rts
