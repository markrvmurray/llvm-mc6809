; RUN: cat %s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution test: 32-bit multiply via __mulsi3.
; Tests the library function directly (no LLVM codegen).
; __mulsi3 ABI: X=a_hi, stack: a_lo, b_hi, b_lo → X=result_hi, stack=result_lo

.include "runtime.inc"

	.text
.include "mulhi3.inc"
.include "mulsi3.inc"

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

;;; mul32 — call __mulsi3(a, b) and print 8-digit hex result
;;; Input: a_hi:a_lo and b_hi:b_lo pushed on stack before call
;;; Stack on entry: [retH][retL][a_hi][a_lo][b_hi][b_lo]
mul32_and_print:
	;; Load args from our caller's stack
	;; S+0,1 = return addr
	;; S+2,3 = a_hi
	;; S+4,5 = a_lo
	;; S+6,7 = b_hi
	;; S+8,9 = b_lo
	leas	-6,s		; allocate __mulsi3 arg space
	;; S+0..5 = arg space
	;; S+6,7 = return addr
	;; S+8,9 = a_hi
	;; S+10,11 = a_lo
	;; S+12,13 = b_hi
	;; S+14,15 = b_lo
	ldd	10,s		; a_lo
	std	,s		; arg slot 0 = a_lo
	ldd	12,s		; b_hi
	std	2,s		; arg slot 1 = b_hi
	ldd	14,s		; b_lo
	std	4,s		; arg slot 2 = b_lo
	ldx	8,s		; X = a_hi
	jsr	__mulsi3
	;; X = result_hi, ,s = result_lo
	jsr	putx		; print result_hi (X)
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx		; print result_lo
	jsr	putnl
	leas	6,s		; clean up arg space
	rts

	.globl	test_main
test_main:
	;; 3 * 7 = 21 = 0x00000015
	ldd	#7		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#3		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK: 00000015

	;; 1000 * 1000 = 1000000 = 0x000F4240
	ldd	#1000		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#1000		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 000F4240

	;; 256 * 256 = 65536 = 0x00010000
	ldd	#256		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#256		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 00010000

	;; 65535 * 65535 = 4294836225 = 0xFFFE0001
	ldd	#0xFFFF		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: FFFE0001

	;; 0 * 99999 = 0
	;; 99999 = 0x0001869F → hi=0x0001, lo=0x869F
	ldd	#0x869F		; b_lo
	pshs	d
	ldd	#0x0001		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 00000000

	;; 1 * 0xDEADBEEF = 0xDEADBEEF (identity)
	;; 0xDEADBEEF → hi=0xDEAD, lo=0xBEEF
	ldd	#0xBEEF		; b_lo
	pshs	d
	ldd	#0xDEAD		; b_hi
	pshs	d
	ldd	#1		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: DEADBEEF

	;; 0x10000 * 0x10000 = 0x100000000 truncated = 0x00000000
	;; 0x10000 → hi=0x0001, lo=0x0000
	ldd	#0		; b_lo
	pshs	d
	ldd	#1		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldd	#1		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 00000000

	;; 12345 * 6789 = 83810205 = 0x04FED79D
	ldd	#6789		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#12345		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 04FED79D

	;; Commutativity: 6789 * 12345 = 83810205 = 0x04FED79D
	ldd	#12345		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#6789		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 04FED79D

	;; Large * large: 0x0100 * 0x01000000 = 0x00000000 (overflow)
	;; Tests cross-word multiplication
	ldd	#0		; b_lo
	pshs	d
	ldd	#0x0100		; b_hi
	pshs	d
	ldd	#0x0100		; a_lo
	pshs	d
	ldd	#0		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 00000000

	;; 0xFFFFFFFF * 0xFFFFFFFF = 0xFFFFFFFE00000001 truncated = 0x00000001
	ldd	#0xFFFF		; b_lo
	pshs	d
	ldd	#0xFFFF		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldd	#0xFFFF		; a_hi
	pshs	d
	jsr	mul32_and_print
	leas	8,s
; CHECK-NEXT: 00000001

	jsr	halt
