; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=200000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Logic and shift instruction execution tests.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; === AND ===
	;; 0xFF & 0x0F = 0x0F
	lda	#0xff
	anda	#0x0f
	jsr	puthex
	jsr	putnl
; CHECK: 0F

	;; 0xA5 & 0x5A = 0x00
	lda	#0xa5
	anda	#0x5a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; === OR ===
	;; 0xA0 | 0x05 = 0xA5
	lda	#0xa0
	ora	#0x05
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: A5

	;; 0x00 | 0x00 = 0x00
	lda	#0x00
	ora	#0x00
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; === XOR ===
	;; 0xFF ^ 0xAA = 0x55
	lda	#0xff
	eora	#0xaa
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 55

	;; XOR self = 0
	lda	#0x42
	eora	#0x42
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; === ASL (arithmetic shift left) ===
	;; 0x21 << 1 = 0x42
	lda	#0x21
	asla
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; 0x80 << 1 = 0x00 (high bit shifted out)
	lda	#0x80
	asla
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; === ASR (arithmetic shift right, preserves sign) ===
	;; 0x84 >> 1 = 0xC2 (sign bit preserved)
	lda	#0x84
	asra
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: C2

	;; 0x42 >> 1 = 0x21 (positive, sign bit stays 0)
	lda	#0x42
	asra
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 21

	;; === LSR (logical shift right, zero-fills) ===
	;; 0x84 >> 1 = 0x42 (zero fill, not sign extend)
	lda	#0x84
	lsra
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; 0x01 >> 1 = 0x00
	lda	#0x01
	lsra
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; === ROL (rotate left through carry) ===
	;; Clear carry, ROL 0x81 = 0x02, carry now set.
	;; Then ROL 0x00 with carry = 0x01.
	;; Do both before printing to preserve carry between them.
	andcc	#0xfe		; clear carry
	lda	#0x81
	rola			; A=0x02, C=1
	pshs	a		; save first result
	lda	#0x00
	rola			; A=0x01 (C rotated into bit0)
	pshs	a		; save second result
	lda	1,s		; first result
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 02

	puls	a		; second result
	jsr	puthex
	leas	1,s		; discard first result
	jsr	putnl
; CHECK-NEXT: 01

	;; === ROR (rotate right through carry) ===
	;; Clear carry, ROR 0x01 = 0x00, carry now set.
	;; Then ROR 0x00 with carry = 0x80.
	andcc	#0xfe		; clear carry
	lda	#0x01
	rora			; A=0x00, C=1
	pshs	a		; save first result
	lda	#0x00
	rora			; A=0x80 (C rotated into bit7)
	pshs	a		; save second result
	lda	1,s		; first result
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	puls	a		; second result
	jsr	puthex
	leas	1,s		; discard first result
	jsr	putnl
; CHECK-NEXT: 80

	;; === BIT (test bits, no store) ===
	;; BITA doesn't change A
	lda	#0x42
	bita	#0x0f		; test low nibble
	jsr	puthex		; A should still be 0x42
	jsr	putnl
; CHECK-NEXT: 42

	;; === Shift memory ===
	;; ASL memory: 0x21 << 1 = 0x42
	lda	#0x21
	sta	0x0100
	asl	0x0100
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; LSR memory: 0x84 >> 1 = 0x42
	lda	#0x84
	sta	0x0100
	lsr	0x0100
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; === CLR (clear) ===
	lda	#0xff
	clra
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; CLR memory
	lda	#0xff
	sta	0x0100
	clr	0x0100
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	rts
