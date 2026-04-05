;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 16-bit multiply via __mulhi3 libcall.

.include "runtime.inc"

	.text
.include "mulhi3.inc"

	.section .rom,"ax",@progbits

;;; putx — print X as 4 hex digits (preserves X)
putx:
	pshs	x
	tfr	x,d
	tfr	a,b
	tfr	b,a
	jsr	puthex		; high byte
	puls	x
	pshs	x
	tfr	x,d
	tfr	b,a
	jsr	puthex		; low byte
	puls	x
	rts

	.globl	test_main
test_main:
	;; 6 * 7 = 42 = 0x002A
	ldd	#7
	pshs	d
	ldx	#6
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK: 002A

	;; 100 * 200 = 20000 = 0x4E20
	ldd	#200
	pshs	d
	ldx	#100
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 4E20

	;; 255 * 255 = 65025 = 0xFE01
	ldd	#0xFF
	pshs	d
	ldx	#0xFF
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FE01

	;; 0 * 12345 = 0
	ldd	#12345
	pshs	d
	ldx	#0
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; 1 * 0xABCD = 0xABCD (identity)
	ldd	#0xABCD
	pshs	d
	ldx	#1
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: ABCD

	;; 256 * 256 = 65536 truncated = 0x0000
	ldd	#256
	pshs	d
	ldx	#256
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; 0x80 * 2 = 0x0100 (carry into high byte)
	ldd	#2
	pshs	d
	ldx	#0x80
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0100

	;; 0x101 * 0x101 = 0x10201 truncated = 0x0201
	ldd	#0x101
	pshs	d
	ldx	#0x101
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0201

	;; Commutativity: 13 * 500 = 6500 = 0x1964
	ldd	#500
	pshs	d
	ldx	#13
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1964

	;; 500 * 13 = 6500 = 0x1964 (same result)
	ldd	#13
	pshs	d
	ldx	#500
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1964

	;; 0xFFFF * 0xFFFF = 0xFFFE0001 truncated = 0x0001
	ldd	#0xFFFF
	pshs	d
	ldx	#0xFFFF
	jsr	test_mul16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	jsr	halt
