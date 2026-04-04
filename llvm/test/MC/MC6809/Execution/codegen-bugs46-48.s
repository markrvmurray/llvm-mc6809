; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-bugs46-48.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution tests for bugs #46 (G_ABS), #47 (null pointer), #48 (LEASi operand).

.include "runtime.inc"

	.section .rom,"ax",@progbits

putx:
	pshs	x,d
	tfr	x,d
	tfr	a,b
	tfr	b,a
	jsr	puthex
	puls	x,d
	pshs	x,d
	tfr	x,d
	tfr	b,a
	jsr	puthex
	puls	x,d
	rts

	.globl	test_main
test_main:
	;; === Bug #46: abs (G_ABS legalization) ===

	;; abs(5) = 5
	ldx	#5
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK: 0005

	;; abs(-3) = 3
	ldx	#-3
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0003

	;; abs(0) = 0
	ldx	#0
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; abs(-1) = 1
	ldx	#-1
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; === Bug #48: 16-bit bitwise via push/pull (LEASi operand fix) ===

	;; or16(0x00FF, 0xFF00) = 0xFFFF
	ldx	#0x00FF
	leas	-2,s
	ldd	#0xFF00
	std	,s
	jsr	test_or16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	;; and16(0x0F0F, 0xFF00) = 0x0F00
	ldx	#0x0F0F
	leas	-2,s
	ldd	#0xFF00
	std	,s
	jsr	test_and16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0F00

	;; xor16(0xAAAA, 0x5555) = 0xFFFF
	ldx	#0xAAAA
	leas	-2,s
	ldd	#0x5555
	std	,s
	jsr	test_xor16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	rts
