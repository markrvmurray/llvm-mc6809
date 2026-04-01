; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 16-bit add functions.
; CC: first i16 arg in X, remaining args on stack. Return in X.

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
	;; add_i16(100, 200) = 300 = 0x012C
	ldd	#200
	pshs	d
	ldx	#100
	jsr	add_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK: 012C

	;; add_i16(0xFFFF, 1) = 0 (overflow)
	ldd	#1
	pshs	d
	ldx	#0xFFFF
	jsr	add_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; add_i16(0x8000, 0x8000) = 0 (overflow)
	ldd	#0x8000
	pshs	d
	ldx	#0x8000
	jsr	add_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; add_i16_const(234) = 234 + 1000 = 1234 = 0x04D2
	ldx	#234
	jsr	add_i16_const
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 04D2

	;; add_i16_chain(100, 200, 300) = 600 = 0x0258
	ldd	#300
	pshs	d
	ldd	#200
	pshs	d
	ldx	#100
	jsr	add_i16_chain
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0258

	;; Commutativity: add_i16(200, 100) = 300
	ldd	#100
	pshs	d
	ldx	#200
	jsr	add_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 012C

	jsr	halt
