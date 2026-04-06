; RUN: cat %s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution test: 32-bit shifts via shiftsi3.inc.
; Tests library functions directly (not via codegen).

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
;;; Stack at entry: [ret:2][func:2][a_hi:2][a_lo:2][shift:2]
;;; S+0=ret  S+2=func  S+4=a_hi  S+6=a_lo  S+8=shift
;;; Library ABI: X = a_hi, [S]=a_lo, B = shift count
shift32_and_print:
	ldb	9,s		; shift count (lo byte of shift word)
	ldd	6,s		; a_lo
	pshs	d		; push a_lo for library (S shifts by 2)
	ldx	6,s		; a_hi (was S+4, now S+6)
	lda	11,s		; reload shift count (was S+9, now S+11)
	tfr	a,b		; B = shift count
	jsr	[4,s]		; call shift function (was S+2, now S+4)
	;; X = result_hi, [S] = result_lo
	ldd	,s
	leas	2,s		; clean pushed a_lo
	pshs	d		; save result_lo (putx clobbers D)
	jsr	putx		; print hi (X)
	puls	d
	tfr	d,x
	jsr	putx		; print lo
	jsr	putnl
	rts

	.globl	test_main
test_main:

	;; shl32(0x12345678, 4) = 0x23456780
	ldd	#4
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#__ashlsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK: 23456780

	;; shl32(1, 16) = 0x00010000
	ldd	#16
	pshs	d
	ldd	#1
	pshs	d
	ldd	#0
	pshs	d
	ldd	#__ashlsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK-NEXT: 00010000

	;; lshr32(0x12345678, 8) = 0x00123456
	ldd	#8
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#__lshrsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK-NEXT: 00123456

	;; lshr32(0x80000000, 1) = 0x40000000
	ldd	#1
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x8000
	pshs	d
	ldd	#__lshrsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK-NEXT: 40000000

	;; ashr32(0x80000000, 4) = 0xF8000000
	ldd	#4
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x8000
	pshs	d
	ldd	#__ashrsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK-NEXT: F8000000

	;; ashr32(0x7FFFFFFF, 16) = 0x00007FFF
	ldd	#16
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#0x7FFF
	pshs	d
	ldd	#__ashrsi3
	pshs	d
	jsr	shift32_and_print
	leas	8,s
; CHECK-NEXT: 00007FFF

	jsr	halt
