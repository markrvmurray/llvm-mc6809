;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 8-bit multiply functions.
; CC: first arg in B, remaining i8 args as 1-byte stack slots
; (matches gcc6809).

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; mul_i8(6, 7) = 42 = 0x2A
	ldb	#7
	pshs	b
	ldb	#6
	jsr	mul_i8
	leas	1,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 2A

	;; mul_i8(0, 5) = 0
	ldb	#5
	pshs	b
	ldb	#0
	jsr	mul_i8
	leas	1,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; mul_i8(15, 17) = 255 = 0xFF
	ldb	#17
	pshs	b
	ldb	#15
	jsr	mul_i8
	leas	1,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; mul_i8_const(6) = 6*7 = 42 = 0x2A
	ldb	#6
	jsr	mul_i8_const
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 2A

	;; mul_i8_chain(2, 3, 7) = 2*3*7 = 42 = 0x2A
	ldb	#7
	pshs	b
	ldb	#3
	pshs	b
	ldb	#2
	jsr	mul_i8_chain
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 2A

	jsr	halt
