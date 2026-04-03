; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: constant shifts (shl, lshr, ashr) for i8 and i16.

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

	;; ===== i16 shifts =====

	;; shl16 0x1234 << 1 = 0x2468
	ldx	#0x1234
	jsr	test_shl16_1
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 2468

	;; shl16 0x8001 << 1 = 0x0002 (overflow)
	ldx	#0x8001
	jsr	test_shl16_1
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; lshr16 0x2468 >> 1 = 0x1234
	ldx	#0x2468
	jsr	test_lshr16_1
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1234

	;; lshr16 0x8000 >> 1 = 0x4000 (logical: 0 into MSB)
	ldx	#0x8000
	jsr	test_lshr16_1
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 4000

	;; ashr16 0x8000 >> 1 = 0xC000 (arithmetic: sign preserved)
	ldx	#0x8000
	jsr	test_ashr16_1
	jsr	putx
	jsr	putnl
; CHECK-NEXT: C000

	;; lshr16 0xABCD >> 4 = 0x0ABC
	ldx	#0xABCD
	jsr	test_lshr16_4
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0ABC

	;; ashr16 0x8000 >> 4 = 0xF800 (sign-extend)
	ldx	#0x8000
	jsr	test_ashr16_4
	jsr	putx
	jsr	putnl
; CHECK-NEXT: F800

	jsr	halt
