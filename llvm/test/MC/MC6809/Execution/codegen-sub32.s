; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sub32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 32-bit subtraction.
; i32 CC: first arg = X (high) + stack (low), second arg on stack (hi, lo).
; Return: X = result high, result low written to first arg's low slot on stack.

.include "runtime.inc"

	.section .rom,"ax",@progbits

;;; putx -- print X as 4 hex digits (preserves X)
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

;;; print32 -- print X:D as 8 hex digits (X=high, D=low)
print32:
	pshs	d		; save low word
	jsr	putx		; print high word (X)
	puls	d
	tfr	d,x
	jsr	putx		; print low word
	jsr	putnl
	rts

	.globl	test_main
test_main:
	;; sub_i32(5, 3) = 2 = 0x00000002
	ldd	#3		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#5		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK: 00000002

	;; sub_i32(0x00010000, 1) = 0x0000FFFF (borrow from high word)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldx	#1		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 0000FFFF

	;; sub_i32(0, 1) = 0xFFFFFFFF (underflow)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: FFFFFFFF

	;; sub_i32(0x80000000, 1) = 0x7FFFFFFF
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldx	#0x8000		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 7FFFFFFF

	;; sub_i32(0xACF13568, 0x9ABCDEF0) = 0x12345678
	ldd	#0xDEF0		; b_lo
	pshs	d
	ldd	#0x9ABC		; b_hi
	pshs	d
	ldd	#0x3568		; a_lo
	pshs	d
	ldx	#0xACF1		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 12345678

	;; sub_i32(100, 100) = 0
	ldd	#100		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#100		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 00000000

	;; sub_i32(0x12345678, 0xACF13568) = 0x65432110
	;; Actually: 0x12345678 - 0xACF13568 = 0x65432110? Let me compute:
	;; 0x12345678 - 0xACF13568 = negative = 0x65432110? No.
	;; 0x12345678 - 0xACF13568 = 0x12345678 + 0x530ECA98 = 0x65432110
	;; Wait: -0xACF13568 = 0x530ECA98. 0x12345678+0x530ECA98 = 0x65432110. Yes.
	ldd	#0x3568		; b_lo
	pshs	d
	ldd	#0xACF1		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	sub_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 65432110

	jsr	halt
