;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 32-bit shifts compiled from LLVM IR.
; i32 CC: first arg = X (high) + stack (low), second arg on stack (hi, lo).
; Return: X = result high, result low on stack.

.include "runtime.inc"
.include "mc6809rt.s"

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

;;; print32 -- print X:D as 8 hex digits (X=high, D=low)
print32:
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
	rts

	.globl	test_main
test_main:
	;; shl32: 1 << 16 = 0x00010000
	ldd	#16		; b_lo (shift amount)
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#1		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	test_shl32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK: 00010000

	;; shl32: 0xFF << 4 = 0x00000FF0
	ldd	#4
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0xFF
	pshs	d
	ldx	#0
	jsr	test_shl32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 00000FF0

	;; lshr32: 0x00010000 >> 8 = 0x00000100
	ldd	#8
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0
	pshs	d
	ldx	#1
	jsr	test_lshr32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: 00000100

	;; ashr32: 0x80000000 >> 16 = 0xFFFF8000 (sign extends)
	ldd	#16
	pshs	d
	ldd	#0
	pshs	d
	ldd	#0
	pshs	d
	ldx	#0x8000
	jsr	test_ashr32
	ldd	,s
	leas	6,s
	jsr	print32
; CHECK-NEXT: FFFF8000

	rts
