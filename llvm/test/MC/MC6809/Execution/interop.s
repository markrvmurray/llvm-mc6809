; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/interop.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; CMOC __gcccall ↔ LLVM interop test.
; CMOC-compiled driver calls LLVM-compiled functions via GCC 6809 ABI.
;
; Tests:
;   llvm_add8(0x30, 0x12) = 0x42      — 8-bit multi-arg
;   llvm_negate8(0xBE) = 0x42          — 8-bit single-arg
;   llvm_identity16(0xBEEF) = 0xBEEF  — 16-bit pass-through via X
;   llvm_identity16(0x1234) = 0x1234   — 16-bit via X (second call)

.include "runtime.inc"
.include "test_interop_all_gas.s"

	.section .rom,"ax",@progbits

; CMOC→LLVM name bridges
	.globl	_llvm_add8
	.set	_llvm_add8, llvm_add8
	.globl	_llvm_negate8
	.set	_llvm_negate8, llvm_negate8
	.globl	_llvm_identity16
	.set	_llvm_identity16, llvm_identity16

	.globl	_puthex
_puthex:
	lda	3,s
	jsr	puthex
	rts
	.globl	_putnl
_putnl:
	jsr	putnl
	rts
	.globl	_halt
_halt:
	jsr	halt
	rts
	.globl	test_main
test_main:
	jsr	_main
	rts

; CHECK: 42
; CHECK-NEXT: 42
; CHECK-NEXT: BEEF
; CHECK-NEXT: 1234
