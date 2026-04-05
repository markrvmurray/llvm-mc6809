;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sret.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sret.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-sret.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: struct return via sret pointer.

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
	;; Allocate 4 bytes on stack for struct Pair result
	leas	-4,s

	;; make_pair(&result, 0x1234, 0x5678)
	;; sret ptr in X, args on stack
	ldd	#0x5678		; y
	pshs	d
	ldd	#0x1234		; x
	pshs	d
	leax	4,s		; X = pointer to result struct
	jsr	make_pair
	leas	4,s		; clean up args

	;; Read and print result.x (offset 0)
	ldd	,s
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK: 1234

	;; Read and print result.y (offset 2)
	ldd	2,s
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 5678

	leas	4,s		; clean up result

	;; Second test: make_pair(&result, 0xBEEF, 0xCAFE)
	leas	-4,s
	ldd	#0xCAFE
	pshs	d
	ldd	#0xBEEF
	pshs	d
	leax	4,s
	jsr	make_pair
	leas	4,s

	ldd	,s
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: BEEF

	ldd	2,s
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: CAFE

	leas	4,s

	jsr	halt
