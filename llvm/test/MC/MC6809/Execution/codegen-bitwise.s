; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-bitwise.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: bitwise AND, OR, XOR for i8.
; NOTE: i16 bitwise emits 6309-only ANDD/ORD/EORD on 6809 (bug #26).

.include "runtime.inc"

	.section .rom,"ax",@progbits

	.globl	test_main
test_main:
	;; i8 calling convention: first arg in B, second in 2-byte stack slot
	;; (value at offset+1, i.e. 3,s inside callee after return addr push)

	;; ===== AND i8 =====

	;; 0xFF & 0x0F = 0x0F
	ldd	#0x0F		; B = second arg (in low byte of D)
	pshs	d		; push 2-byte slot
	ldb	#0xFF		; B = first arg
	jsr	test_and8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 0F

	;; 0xA5 & 0x5A = 0x00
	ldd	#0x5A
	pshs	d
	ldb	#0xA5
	jsr	test_and8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; 0xF0 & 0xFF = 0xF0
	ldd	#0xFF
	pshs	d
	ldb	#0xF0
	jsr	test_and8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: F0

	;; ===== OR i8 =====

	;; 0xA0 | 0x05 = 0xA5
	ldd	#0x05
	pshs	d
	ldb	#0xA0
	jsr	test_or8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: A5

	;; 0x00 | 0x00 = 0x00
	ldd	#0x00
	pshs	d
	ldb	#0x00
	jsr	test_or8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; 0x0F | 0xF0 = 0xFF
	ldd	#0xF0
	pshs	d
	ldb	#0x0F
	jsr	test_or8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; ===== XOR i8 =====

	;; 0xFF ^ 0xAA = 0x55
	ldd	#0xAA
	pshs	d
	ldb	#0xFF
	jsr	test_xor8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 55

	;; 0xAA ^ 0xAA = 0x00 (self-XOR = 0)
	ldd	#0xAA
	pshs	d
	ldb	#0xAA
	jsr	test_xor8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; 0x00 ^ 0xFF = 0xFF (NOT via XOR)
	ldd	#0xFF
	pshs	d
	ldb	#0x00
	jsr	test_xor8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	jsr	halt
