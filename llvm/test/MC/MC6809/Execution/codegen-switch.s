;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-switch.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-switch.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-switch.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: switch with 5 cases (jump table).

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

	;; switch5(0) = 100 = 0x0064
	ldx	#0
	jsr	switch5
	jsr	putx
	jsr	putnl
; CHECK: 0064

	;; switch5(1) = 200 = 0x00C8
	ldx	#1
	jsr	switch5
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00C8

	;; switch5(4) = 500 = 0x01F4
	ldx	#4
	jsr	switch5
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 01F4

	;; switch5(5) = 999 = 0x03E7 (default)
	ldx	#5
	jsr	switch5
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 03E7

	;; switch5(0xFFFF) = 999 = 0x03E7 (default, large value)
	ldx	#0xFFFF
	jsr	switch5
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 03E7

	jsr	halt
