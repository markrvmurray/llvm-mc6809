; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sub.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: subtraction.
; CC: first arg in B, remaining as packed bytes on stack.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; sub_simple(0x50, 0x08) = 0x48
	lda	#0x08
	pshs	a
	ldb	#0x50
	jsr	sub_simple
	leas	1,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 48

	;; sub_simple(0x42, 0x00) = 0x42
	lda	#0x00
	pshs	a
	ldb	#0x42
	jsr	sub_simple
	leas	1,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; sub_s_i8_consts(0,0,0,0,0) = 5
	lda	#0
	pshs	a
	pshs	a
	pshs	a
	pshs	a
	ldb	#0
	jsr	sub_s_i8_consts
	leas	4,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 05

	;; sub_s_i8_consts(1,1,1,1,1) = 0
	lda	#1
	pshs	a
	pshs	a
	pshs	a
	pshs	a
	ldb	#1
	jsr	sub_s_i8_consts
	leas	4,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	rts
