; RUN: cat %s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution test: 32-bit shifts via shiftsi3.inc.
; Tests library functions directly (like codegen-mul32.s).

.include "runtime.inc"

	.text
.include "shiftsi3.inc"

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

;;; shift32_and_print — call a 32-bit shift and print result
;;; Stack: [ret:2][func:2][a_hi:2][a_lo:2][b_hi:2][b_lo:2]
shift32_and_print:
	leas	-6,s
	ldd	12,s		; a_lo
	std	,s
	ldd	14,s		; b_hi (0 for shifts)
	std	2,s
	ldd	16,s		; b_lo (shift amount)
	std	4,s
	ldx	10,s		; X = a_hi
	jsr	[8,s]		; call shift function
	;; X = result_lo, ,s = result_hi
	pshs	x
	ldd	2,s		; result_hi
	tfr	d,x
	jsr	putx		; print hi
	puls	x
	jsr	putx		; print lo
	jsr	putnl
	leas	6,s
	rts

	.globl	test_main
test_main:

	;; shl32(0x12345678, 4) = 0x23456780
	ldd	#4
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#__ashlsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK: 23456780

	;; shl32(1, 16) = 0x00010000
	ldd	#16
	pshs	d
	ldd	#0
	pshs	d
	ldd	#1
	pshs	d
	ldd	#0
	pshs	d
	ldd	#__ashlsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK-NEXT: 00010000

	;; lshr32(0x12345678, 8) = 0x00123456
	ldd	#8
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#__lshrsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK-NEXT: 00123456

	;; lshr32(0x80000000, 1) = 0x40000000
	ldd	#1
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x8000
	pshs	d
	ldd	#__lshrsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK-NEXT: 40000000

	;; ashr32(0x80000000, 4) = 0xF8000000
	ldd	#4
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x8000
	pshs	d
	ldd	#__ashrsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK-NEXT: F8000000

	;; ashr32(0x7FFFFFFF, 16) = 0x00007FFF
	ldd	#16
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#0x7FFF
	pshs	d
	ldd	#__ashrsi3
	pshs	d
	jsr	shift32_and_print
	leas	10,s
; CHECK-NEXT: 00007FFF

	jsr	halt
