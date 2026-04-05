;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-div16.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-div16.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-div16.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-div16.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 16-bit division and remainder via libcalls.
; Generated code uses BSR for libcalls; sed converts to JSR for range.

.include "runtime.inc"

	.text
.include "divhi.inc"

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

	;; ===== Unsigned division (udiv) =====

	;; 100 / 10 = 10
	ldd	#10
	pshs	d
	ldx	#100
	jsr	test_udiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK: 000A

	;; 0xFFFF / 256 = 255
	ldd	#256
	pshs	d
	ldx	#0xFFFF
	jsr	test_udiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00FF

	;; 7 / 2 = 3
	ldd	#2
	pshs	d
	ldx	#7
	jsr	test_udiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0003

	;; 0 / 5 = 0
	ldd	#5
	pshs	d
	ldx	#0
	jsr	test_udiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ===== Unsigned remainder (urem) =====

	;; 100 % 10 = 0
	ldd	#10
	pshs	d
	ldx	#100
	jsr	test_urem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; 7 % 2 = 1
	ldd	#2
	pshs	d
	ldx	#7
	jsr	test_urem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; 255 % 16 = 15
	ldd	#16
	pshs	d
	ldx	#255
	jsr	test_urem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 000F

	;; 1000 % 7 = 6
	ldd	#7
	pshs	d
	ldx	#1000
	jsr	test_urem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0006

	;; ===== Signed division (sdiv) =====

	;; 100 / 10 = 10
	ldd	#10
	pshs	d
	ldx	#100
	jsr	test_sdiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 000A

	;; -100 / 10 = -10 (0xFFF6)
	ldd	#10
	pshs	d
	ldx	#0xFF9C		; -100
	jsr	test_sdiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFF6

	;; 100 / -10 = -10 (0xFFF6)
	ldd	#0xFFF6		; -10
	pshs	d
	ldx	#100
	jsr	test_sdiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFF6

	;; -100 / -10 = 10
	ldd	#0xFFF6		; -10
	pshs	d
	ldx	#0xFF9C		; -100
	jsr	test_sdiv16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 000A

	;; ===== Signed remainder (srem) =====

	;; 7 % 2 = 1
	ldd	#2
	pshs	d
	ldx	#7
	jsr	test_srem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; -7 % 2 = -1 (0xFFFF)
	ldd	#2
	pshs	d
	ldx	#0xFFF9		; -7
	jsr	test_srem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	;; 7 % -2 = 1
	ldd	#0xFFFE		; -2
	pshs	d
	ldx	#7
	jsr	test_srem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; -7 % -2 = -1 (0xFFFF)
	ldd	#0xFFFE		; -2
	pshs	d
	ldx	#0xFFF9		; -7
	jsr	test_srem16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	jsr	halt
