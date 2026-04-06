;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-labs.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-labs.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-labs.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-labs.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; KNOWN-FAIL: this test exists to keep bug #61 visible.
; labs(-3) currently returns 0xFFFFFFFD (= -3) instead of 0x00000003.
; labs(0) and labs(5) work correctly. The test fails on labs(-3).
; Once bug #61 is fixed, the CHECK-NEXT line for labs(-3) below will
; pass and this whole file can move into codegen-picolibc.s.

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

	.globl	test_main
test_main:
	;; labs(5) = 5  (positive — works correctly)
	leas	-2,s
	ldd	#5		; lo
	std	,s
	ldx	#0		; hi
	jsr	test_labs
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK: 00000005

	;; labs(0) = 0
	leas	-2,s
	ldd	#0		; lo
	std	,s
	ldx	#0		; hi
	jsr	test_labs
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000000

	;; labs(-3) = 3  (negative — BUG #61: returns FFFFFFFD)
	;; -3 in i32 = 0xFFFFFFFD → hi=0xFFFF, lo=0xFFFD
	leas	-2,s
	ldd	#0xFFFD		; lo
	std	,s
	ldx	#0xFFFF		; hi
	jsr	test_labs
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000003

	rts
