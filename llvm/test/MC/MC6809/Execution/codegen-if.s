; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: if/branch.
; CC: first arg in B, remaining as 16-bit words on stack.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; if_s8(5, 3) = 5
	clra
	ldb	#3
	pshs	b,a
	ldb	#5
	jsr	if_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 05

	;; if_s8(3, 5) = 5
	clra
	ldb	#5
	pshs	b,a
	ldb	#3
	jsr	if_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 05

	;; if_s8(3, 3) = 3
	clra
	ldb	#3
	pshs	b,a
	ldb	#3
	jsr	if_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 03

	;; if_s8(-1, 1) = 1
	clra
	ldb	#1
	pshs	b,a
	ldb	#0xff
	jsr	if_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	;; if_s8(1, -1) = 1
	lda	#0xff
	ldb	#0xff
	pshs	b,a
	ldb	#1
	jsr	if_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	rts
