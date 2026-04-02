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
; Codegen execution test: bitwise AND, OR, XOR for i8 and i16.

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
	;; i8 CC: first arg in B, second in 2-byte stack slot (value at +1)
	;; i16 CC: first arg in X, second in 2-byte stack slot

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

	;; ===== AND i16 =====

	;; 0xFF00 & 0x00FF = 0x0000
	ldd	#0x00FF
	pshs	d
	ldx	#0xFF00
	jsr	test_and16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; 0xABCD & 0xFF00 = 0xAB00
	ldd	#0xFF00
	pshs	d
	ldx	#0xABCD
	jsr	test_and16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: AB00

	;; ===== OR i16 =====

	;; 0xFF00 | 0x00FF = 0xFFFF
	ldd	#0x00FF
	pshs	d
	ldx	#0xFF00
	jsr	test_or16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	;; 0x1234 | 0x0000 = 0x1234
	ldd	#0x0000
	pshs	d
	ldx	#0x1234
	jsr	test_or16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1234

	;; ===== XOR i16 =====

	;; 0xFFFF ^ 0xAAAA = 0x5555
	ldd	#0xAAAA
	pshs	d
	ldx	#0xFFFF
	jsr	test_xor16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 5555

	;; 0x1234 ^ 0x1234 = 0x0000
	ldd	#0x1234
	pshs	d
	ldx	#0x1234
	jsr	test_xor16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	jsr	halt
