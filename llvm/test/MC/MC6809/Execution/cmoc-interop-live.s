; RUN: mkdir -p %t.dir && cd %t.dir \
; RUN:   && cp %S/Inputs/cmoc_driver.c driver.c \
; RUN:   && cmoc -S driver.c 2>/dev/null \
; RUN:   && python3 %S/Inputs/cmoc2gas.py < driver.s > cmoc-gas.s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/cmoc-interop.ll -o %t-llvm-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-llvm-raw.s > %t-llvm.s
; RUN: cat %s %t.dir/cmoc-gas.s %t-llvm.s | llvm-mc -triple=mc6809 \
; RUN:   -I %S/Inputs --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim, cmoc
;
; Live CMOC↔LLVM interop: CMOC driver calls LLVM functions via __gcccall.

.include "runtime.inc"
.include "mc6809rt.s"

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

;;; Bridge: CMOC __gcccall puthex16(X) → our putx (X)
	.globl	_puthex16
_puthex16:
	jsr	putx
	rts

;;; Bridge: CMOC __gcccall putnl() → our putnl
	.globl	_putnl
_putnl:
	jsr	putnl
	rts

;;; Bridge: CMOC __gcccall halt() → our halt
	.globl	_halt
_halt:
	jsr	halt
	rts

;;; Entry: CMOC _main is the test driver
	.globl	test_main
test_main:
	jsr	_main
	jsr	halt

;;; Bridge CMOC _llvm_* → LLVM llvm_*
	.globl	_llvm_sum
	.set	_llvm_sum, llvm_sum
	.globl	_llvm_mul
	.set	_llvm_mul, llvm_mul

; CHECK: 0007
; CHECK-NEXT: 68AC
; CHECK-NEXT: 002A
