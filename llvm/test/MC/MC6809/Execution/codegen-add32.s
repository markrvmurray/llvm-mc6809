; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 32-bit addition.
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
	;; add_i32(1, 2) = 3 = 0x00000003
	ldd	#2		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#1		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	add_i32
	;; X = result_hi, ,s = result_lo
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK: 00000003

	;; add_i32(0x0000FFFF, 1) = 0x00010000 (carry into high word)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	add_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 00010000

	;; add_i32(0x7FFFFFFF, 1) = 0x80000000 (signed overflow)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldx	#0x7FFF		; a_hi
	jsr	add_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 80000000

	;; add_i32(0xFFFFFFFF, 1) = 0x00000000 (unsigned overflow)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldx	#0xFFFF		; a_hi
	jsr	add_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 00000000

	;; add_i32(0x12345678, 0x9ABCDEF0) = 0xACF13568
	ldd	#0xDEF0		; b_lo
	pshs	d
	ldd	#0x9ABC		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	add_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: ACF13568

	;; add_i32_const(1) = 1 + 30000 = 30001 = 0x00007531
	ldd	#1		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	add_i32_const
	ldd	,s
	leas	2,s
	jsr	print32
; CHECK-NEXT: 00007531

	;; add_i32_const(0xFFFF0000) = 0xFFFF0000 + 30000
	;; = 0xFFFF0000 + 0x00007530 = 0xFFFF7530
	ldd	#0		; a_lo
	pshs	d
	ldx	#0xFFFF		; a_hi
	jsr	add_i32_const
	ldd	,s
	leas	2,s
	jsr	print32
; CHECK-NEXT: FFFF7530

	;; Commutativity: add_i32(0x9ABCDEF0, 0x12345678) = 0xACF13568
	ldd	#0x5678		; b_lo
	pshs	d
	ldd	#0x1234		; b_hi
	pshs	d
	ldd	#0xDEF0		; a_lo
	pshs	d
	ldx	#0x9ABC		; a_hi
	jsr	add_i32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: ACF13568

	jsr	halt
