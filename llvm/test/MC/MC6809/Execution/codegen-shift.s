; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: constant shifts (shl, lshr, ashr) for i8.

.include "runtime.inc"

	.section .rom,"ax",@progbits

	.globl	test_main
test_main:

	;; shl 0x42 << 1 = 0x84
	ldb	#0x42
	jsr	test_shl8_1
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 84

	;; shl 0x0F << 4 = 0xF0
	ldb	#0x0F
	jsr	test_shl8_4
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: F0

	;; lshr 0x84 >> 1 = 0x42
	ldb	#0x84
	jsr	test_lshr8_1
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; lshr 0xF0 >> 4 = 0x0F
	ldb	#0xF0
	jsr	test_lshr8_4
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 0F

	;; ashr 0x80 >> 1 = 0xC0 (sign-extend: bit 7 preserved)
	ldb	#0x80
	jsr	test_ashr8_1
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: C0

	;; ashr 0x40 >> 1 = 0x20 (positive, same as lshr)
	ldb	#0x40
	jsr	test_ashr8_1
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 20

	;; ashr 0x80 >> 4 = 0xF8 (sign-extend over 4 bits)
	ldb	#0x80
	jsr	test_ashr8_4
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: F8

	;; ashr 0x7F >> 4 = 0x07 (positive)
	ldb	#0x7F
	jsr	test_ashr8_4
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 07

	jsr	halt
