; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; 16-bit calling convention validation.
;
; Tests:
;   1. identity16(0xBEEF) = 0xBEEF  — 16-bit pass-through
;   2. add16(0x1234, 0x4321) = 0x5555 — 16-bit addition
;   3. sub16(0x8000, 0x0001) = 0x7FFF — 16-bit subtraction

.include "runtime.inc"
.include "cc_tests16_gas.s"
.include "cc_test16_funcs.s"

	.section .rom,"ax",@progbits

	.globl	_puthex
_puthex:
	lda	3,s
	jsr	puthex
	rts

	.globl	_putnl
_putnl:
	jsr	putnl
	rts

	.globl	_halt
_halt:
	jsr	halt
	rts

	.globl	test_main
test_main:
	jsr	_main
	rts

; CHECK: BEEF
; CHECK-NEXT: 5555
; CHECK-NEXT: 7FFF
