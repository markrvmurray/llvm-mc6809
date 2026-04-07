;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-varargs.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-varargs.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-varargs.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
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

	;; Varargs CC for `T fn(T n, ...)` — gcc6809 __gcccall convention:
	;;   - ALL args (including the named ones) go on the stack starting
	;;     at SP+0 above the return address, in declaration order, all
	;;     16-bit slots.
	;;   - Return value in X (regular RetCC, unchanged for variadic fns).
	;; Caller is responsible for cleaning up the entire arg stack region.

	;; sum_va2(2, 100, 200) = 300 = 0x012C
	leas	-6,s
	ldd	#2
	std	,s		; n (named arg)
	ldd	#100
	std	2,s		; first vararg
	ldd	#200
	std	4,s		; second vararg
	jsr	sum_va2
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK: 012C

	;; sum_va2(2, 0x1234, 0x5678) = 0x68AC
	leas	-6,s
	ldd	#2
	std	,s
	ldd	#0x1234
	std	2,s
	ldd	#0x5678
	std	4,s
	jsr	sum_va2
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 68AC

	;; ===== sum_va4: 4 varargs =====

	;; sum_va4(4, 10, 20, 30, 40) = 100 = 0x0064
	leas	-10,s
	ldd	#4
	std	,s
	ldd	#10
	std	2,s
	ldd	#20
	std	4,s
	ldd	#30
	std	6,s
	ldd	#40
	std	8,s
	jsr	sum_va4
	leas	10,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0064

	;; sum_va4(4, 0x1000, 0x0200, 0x0030, 0x0004) = 0x1234
	leas	-10,s
	ldd	#4
	std	,s
	ldd	#0x1000
	std	2,s
	ldd	#0x0200
	std	4,s
	ldd	#0x0030
	std	6,s
	ldd	#0x0004
	std	8,s
	jsr	sum_va4
	leas	10,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1234

	;; ===== get_nth_va: skip to Nth arg =====

	;; get_nth_va(1, 0xAA, 0xBB, 0xCC, 0xDD) = 0xAA (first)
	leas	-10,s
	ldd	#1
	std	,s
	ldd	#0xAA
	std	2,s
	ldd	#0xBB
	std	4,s
	ldd	#0xCC
	std	6,s
	ldd	#0xDD
	std	8,s
	jsr	get_nth_va
	leas	10,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00AA

	;; get_nth_va(4, 0xAA, 0xBB, 0xCC, 0xDD) = 0xDD (fourth)
	leas	-10,s
	ldd	#4
	std	,s
	ldd	#0xAA
	std	2,s
	ldd	#0xBB
	std	4,s
	ldd	#0xCC
	std	6,s
	ldd	#0xDD
	std	8,s
	jsr	get_nth_va
	leas	10,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00DD

	jsr	halt
