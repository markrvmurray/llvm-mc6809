; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: %hd6309sim %t.elf | FileCheck %s
; REQUIRES: hd6309
;
; HD6309-only LDQ/STQ exercise the HD6309 lit infrastructure
; (runtime6309.inc + %hd6309sim: usim by default, MAME with
; MC6809_HD6309_SIM=mame).

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
