; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Calling convention validation: CMOC-compiled driver (known-good)
; calls hand-written test functions following CMOC convention.
;
; CMOC convention:
;   Args: 16-bit words pushed on stack, right-to-left, caller cleanup
;   Frame: pshs u / leau ,s / ... / leas ,u / puls u,pc
;   Return: 8-bit in B, 16-bit in D
;   Stack after frame setup: U+4=arg1, U+6=arg2, U+8=arg3, ...
;   (arg low byte at odd offset: 5,u / 7,u / 9,u)

.include "runtime.inc"

; CMOC-compiled driver (via cmoc2gas.py)
.include "cc_tests2_gas.s"

; Test functions (hand-written, CMOC calling convention)
.include "cc_test_funcs.s"

; Bridge: CMOC _puthex/_putnl/_halt → our runtime
	.section .rom,"ax",@progbits

	.globl	_puthex
_puthex:
	lda	3,s		; arg low byte (S+0=ret, S+2=arg16)
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

; CHECK: 42
; CHECK-NEXT: 42
; CHECK-NEXT: 42
; CHECK-NEXT: 42
; CHECK-NEXT: 78
