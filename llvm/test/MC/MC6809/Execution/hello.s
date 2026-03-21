; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=100000 %t.hex | FileCheck %s
; REQUIRES: usim

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; Print "Hi!" character by character (workaround: ldx #symbol
	;; and leax symbol,pc have encoding issues with relocations)
	lda	#0x48		; 'H'
	jsr	putchar
	lda	#0x69		; 'i'
	jsr	putchar
	lda	#0x21		; '!'
	jsr	putchar
	jsr	putnl
	rts

; CHECK: Hi!
