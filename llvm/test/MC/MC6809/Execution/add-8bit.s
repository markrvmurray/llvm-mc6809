; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=100000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; 8-bit add instruction execution tests.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; 0x30 + 0x12 = 0x42
	lda	#0x30
	adda	#0x12
	jsr	puthex
	jsr	putnl

	;; 0xFF + 0x01 = 0x00 (8-bit wrap)
	lda	#0xff
	adda	#0x01
	jsr	puthex
	jsr	putnl

	;; 0x7F + 0x01 = 0x80
	lda	#0x7f
	adda	#0x01
	jsr	puthex
	jsr	putnl

	rts

; CHECK: 42
; CHECK-NEXT: 00
; CHECK-NEXT: 80
