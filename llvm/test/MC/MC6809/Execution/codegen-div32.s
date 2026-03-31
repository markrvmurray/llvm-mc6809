; RUN: cat %s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution test: 32-bit division/remainder via divsi.inc.
; Tests library functions directly (like codegen-mul32.s).
; ABI: X=a_hi, stack: a_lo, b_hi, b_lo → X=result_lo, stack=result_hi

.include "runtime.inc"

	.text
.include "divsi.inc"

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

;;; div32_and_print — call a 32-bit div/rem function and print result
;;; Stack on entry: [ret:2][func:2][a_hi:2][a_lo:2][b_hi:2][b_lo:2]
;;; func = address of __udivsi3/__divsi3/__umodsi3/__modsi3
div32_and_print:
	; S+0,1  = return addr (to test_main)
	; S+2,3  = func ptr
	; S+4,5  = a_hi
	; S+6,7  = a_lo
	; S+8,9  = b_hi
	; S+10,11 = b_lo
	leas	-6,s		; allocate arg space for libcall
	; S+0..5  = arg space
	; S+6,7   = return addr
	; S+8,9   = func ptr
	; S+10,11 = a_hi
	; S+12,13 = a_lo
	; S+14,15 = b_hi
	; S+16,17 = b_lo
	ldd	12,s		; a_lo
	std	,s
	ldd	14,s		; b_hi
	std	2,s
	ldd	16,s		; b_lo
	std	4,s
	ldx	10,s		; X = a_hi
	jsr	[8,s]		; indirect call through func ptr
	; X = result_lo, ,s = result_hi
	pshs	x		; save result_lo
	ldd	2,s		; result_hi
	tfr	d,x
	jsr	putx		; print hi word
	puls	x
	jsr	putx		; print lo word
	jsr	putnl
	leas	6,s		; clean up arg space
	rts

	.globl	test_main
test_main:

	;; ===== Unsigned division (__udivsi3) =====

	;; 100 / 10 = 10 = 0x0000000A
	ldd	#10		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#100		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	ldd	#__udivsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK: 0000000A

	;; 1000000 / 1000 = 1000 = 0x000003E8
	;; 1000000 = 0x000F4240
	ldd	#1000		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0x4240		; a_lo
	pshs	d
	ldd	#0x000F	; a_hi
	pshs	d
	ldd	#__udivsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 000003E8

	;; 0xFFFFFFFF / 0x10000 = 0x0000FFFF
	ldd	#0		; b_lo
	pshs	d
	ldd	#1		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldd	#0xFFFF		; a_hi
	pshs	d
	ldd	#__udivsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 0000FFFF

	;; ===== Unsigned remainder (__umodsi3) =====

	;; 1000000 % 1000 = 0 (exact)
	ldd	#1000
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x4240
	pshs	d
	ldd	#0x000F
	pshs	d
	ldd	#__umodsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 00000000

	;; 1000001 % 1000 = 1
	;; 1000001 = 0x000F4241
	ldd	#1000
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0x4241
	pshs	d
	ldd	#0x000F
	pshs	d
	ldd	#__umodsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 00000001

	;; 0xFFFFFFFF % 0x10000 = 0x0000FFFF
	ldd	#0
	pshs	d
	ldd	#1
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#__umodsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 0000FFFF

	;; ===== Signed division (__divsi3) =====

	;; 1000000 / -1000 = -1000
	;; -1000 = 0xFFFFFC18
	;; -1000 result = 0xFFFFFC18
	ldd	#0xFC18		; b_lo (-1000)
	pshs	d
	ldd	#0xFFFF		; b_hi
	pshs	d
	ldd	#0x4240		; a_lo (1000000)
	pshs	d
	ldd	#0x000F		; a_hi
	pshs	d
	ldd	#__divsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: FFFFFC18

	;; -1000000 / -1000 = 1000 = 0x000003E8
	;; -1000000 = 0xFFF0BDC0
	ldd	#0xFC18		; b_lo (-1000)
	pshs	d
	ldd	#0xFFFF		; b_hi
	pshs	d
	ldd	#0xBDC0		; a_lo (-1000000)
	pshs	d
	ldd	#0xFFF0		; a_hi
	pshs	d
	ldd	#__divsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 000003E8

	;; ===== Signed remainder (__modsi3) =====

	;; 7 % -2 = 1
	ldd	#0xFFFE		; b_lo (-2)
	pshs	d
	ldd	#0xFFFF		; b_hi
	pshs	d
	ldd	#7		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	ldd	#__modsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: 00000001

	;; -7 % 2 = -1 = 0xFFFFFFFF
	;; -7 = 0xFFFFFFF9
	ldd	#2		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFF9		; a_lo (-7)
	pshs	d
	ldd	#0xFFFF		; a_hi
	pshs	d
	ldd	#__modsi3
	pshs	d
	jsr	div32_and_print
	leas	10,s
; CHECK-NEXT: FFFFFFFF

	jsr	halt
