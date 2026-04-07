; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-data-init.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: initialized .data globals with ROM→RAM copy.

.include "runtime.inc"

	.section .rom,"ax",@progbits

;;; putx — print X as 4 hex digits (preserves X)
putx:
	pshs	x
	tfr	x,d
	tfr	a,b
	tfr	b,a
	jsr	puthex
	puls	x
	pshs	x
	tfr	x,d
	tfr	b,a
	jsr	puthex
	puls	x
	rts

	.globl	test_main
test_main:

	;; get_counter() initially = 42 = 0x002A (from .data init)
	jsr	get_counter
	jsr	putx
	jsr	putnl
; CHECK: 002A

	;; get_flag() = 1 (from .data init)
	jsr	get_flag
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	;; inc_counter() then get_counter() = 43 = 0x002B
	jsr	inc_counter
	jsr	get_counter
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 002B

	;; inc_counter() twice more → 45 = 0x002D
	jsr	inc_counter
	jsr	inc_counter
	jsr	get_counter
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 002D

	jsr	halt
