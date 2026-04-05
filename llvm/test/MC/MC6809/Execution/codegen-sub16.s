;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sub16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sub16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sub16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 16-bit subtract functions.
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
	;; sub_i16(500, 200) = 300 = 0x012C
	ldd	#200
	pshs	d
	ldx	#500
	jsr	sub_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK: 012C

	;; sub_i16(0, 1) = 0xFFFF (-1 unsigned)
	ldd	#1
	pshs	d
	ldx	#0
	jsr	sub_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	;; sub_i16(1000, 1000) = 0
	ldd	#1000
	pshs	d
	ldx	#1000
	jsr	sub_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; sub_i16_const(1234) = 1234 - 100 = 1134 = 0x046E
	ldx	#1234
	jsr	sub_i16_const
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 046E

	;; sub_i16(0x8000, 1) = 0x7FFF
	ldd	#1
	pshs	d
	ldx	#0x8000
	jsr	sub_i16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 7FFF

	jsr	halt
