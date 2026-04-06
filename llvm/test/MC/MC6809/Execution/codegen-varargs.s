; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-varargs.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: varargs (va_start + va_arg).

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

	;; sum_va2(2, 100, 200) = 300 = 0x012C
	;; Varargs CC: all args on stack, return via stack slot [S+0]
	ldd	#200		; arg 3
	pshs	d
	ldd	#100		; arg 2
	pshs	d
	ldd	#2		; n (arg 1)
	pshs	d
	jsr	sum_va2
	ldd	,s		; result from stack slot
	tfr	d,x
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK: 012C

	;; sum_va2(2, 0x1234, 0x5678) = 0x68AC
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#2
	pshs	d
	jsr	sum_va2
	ldd	,s
	tfr	d,x
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 68AC

	jsr	halt
