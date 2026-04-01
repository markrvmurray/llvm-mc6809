; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-loop.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: loop control flow (PHI, back-edge, exit).
; NOTE: Loops with 2+ live i16 values have register pressure bugs
; (flag clobbering between compare and branch). This tests the
; single-live-value case which works correctly.

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

	;; countdown(5) = 5 (loops 5 times, returns original n)
	ldx	#5
	jsr	countdown
	jsr	putx
	jsr	putnl
; CHECK: 0005

	;; countdown(1) = 1 (loops once)
	ldx	#1
	jsr	countdown
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; countdown(0) = 0 (loop body never executes)
	ldx	#0
	jsr	countdown
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; countdown(256) = 256 (tests larger iteration count)
	ldx	#256
	jsr	countdown
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0100

	;; countdown(-1) = -1 = 0xFFFF (n <= 0, loop body skipped)
	ldx	#0xFFFF
	jsr	countdown
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	jsr	halt
