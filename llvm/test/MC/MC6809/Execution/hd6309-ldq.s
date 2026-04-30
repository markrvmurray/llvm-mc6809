; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: %mamellvm6309 %t.elf | FileCheck %s
; REQUIRES: mame, hd6309
;
; Bug #194 Phase A sentinel: HD6309-only LDQ/STQ exercise the full
; lit infrastructure (runtime6309.inc + %mamellvm6309 wrapper).
; usim09batch traps on LDMD/LDQ, so this test is gated to MAME only.

.include "runtime6309.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; AQ = $12345678 (BE: A=12, B=34, E=56, F=78).
	ldq	#0x12345678
	;; STQ stores AQ in big-endian order: 12 34 56 78.
	stq	scratch
	;; Echo the four bytes as hex.
	lda	scratch
	jsr	puthex
	lda	scratch+1
	jsr	puthex
	lda	scratch+2
	jsr	puthex
	lda	scratch+3
	jsr	puthex
	jsr	putnl
	rts

	.section .ram,"aw",@nobits
scratch:
	.skip	4

; CHECK: 12345678
