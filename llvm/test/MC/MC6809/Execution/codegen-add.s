;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 8-bit add functions.
; CC: first arg in B, remaining args as 16-bit words on stack.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; add_s_i8(1, 2, 3, 4, 5) = 15 = 0x0F
	;; Push as 16-bit words (high byte 0, low byte = value)
	clra
	ldb	#5
	pshs	b,a
	ldb	#4
	pshs	b,a
	ldb	#3
	pshs	b,a
	ldb	#2
	pshs	b,a
	ldb	#1		; first arg in B
	jsr	add_s_i8
	leas	8,s		; clean up 4 x 2-byte words
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 0F

	;; add_s_i8(0x10, 0x10, 0x10, 0x10, 0x02) = 0x42
	clra
	ldb	#0x02
	pshs	b,a
	ldb	#0x10
	pshs	b,a
	pshs	b,a
	pshs	b,a
	ldb	#0x10
	jsr	add_s_i8
	leas	8,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; add_s_i8_consts(0x10, 0x10, 0x10, 0x0D) = 0x42
	clra
	ldb	#0x0d
	pshs	b,a
	ldb	#0x10
	pshs	b,a
	pshs	b,a
	ldb	#0x10
	jsr	add_s_i8_consts
	leas	6,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	rts
