; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=200000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Arithmetic instruction execution tests.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; === 8-bit subtract ===
	;; 0x50 - 0x08 = 0x48
	lda	#0x50
	suba	#0x08
	jsr	puthex
	jsr	putnl
; CHECK: 48

	;; === 8-bit subtract with borrow ===
	;; 0x00 - 0x01 = 0xFF (wraps, sets carry)
	;; then 0x10 - 0x00 - carry = 0x0F
	lda	#0x00
	suba	#0x01		; A=0xFF, C=1
	lda	#0x10
	sbca	#0x00		; A=0x10-0x00-1=0x0F
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 0F

	;; === MUL (A * B → D) ===
	;; 7 * 6 = 42
	lda	#7
	ldb	#6
	mul			; D = A*B = 42 = 0x002A
	tfr	b,a		; A = low byte
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 2A

	;; === MUL large ===
	;; 0xFF * 0xFF = 0xFE01
	lda	#0xff
	ldb	#0xff
	mul			; D = 0xFE01
	tfr	a,b		; save high byte
	tfr	b,a
	jsr	puthex		; print high byte
	lda	#0xff
	ldb	#0xff
	mul
	tfr	b,a
	jsr	puthex		; print low byte
	jsr	putnl
; CHECK-NEXT: FE01

	;; === INC / DEC accumulator ===
	;; inca: 0x41 + 1 = 0x42
	lda	#0x41
	inca
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; decb: 0x43 - 1 = 0x42
	ldb	#0x43
	decb
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; === INC / DEC memory ===
	;; Store 0x41 at RAM, inc it, read back
	lda	#0x41
	sta	0x0100
	inc	0x0100
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; === NEG accumulator ===
	;; nega: -(0x01) = 0xFF
	lda	#0x01
	nega
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; nega: -(0xBE) = 0x42
	lda	#0xbe
	nega
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 42

	;; === NEG memory ===
	lda	#0x01
	sta	0x0100
	neg	0x0100
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; === COM (ones complement) ===
	;; coma: ~0x55 = 0xAA
	lda	#0x55
	coma
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: AA

	;; === CMP (compare, no store) ===
	;; cmpa sets flags but doesn't change A
	lda	#0x42
	cmpa	#0x42		; Z=1
	jsr	puthex		; should still print 42
	jsr	putnl
; CHECK-NEXT: 42

	;; === ADDD (16-bit add) ===
	;; 0x1234 + 0xEDCC = 0x10000 → 0x0000 (wraps)
	ldd	#0x1234
	addd	#0xedcc
	tfr	a,b
	tfr	b,a
	jsr	puthex
	ldd	#0x1234
	addd	#0xedcc
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 0000

	;; === SUBD (16-bit sub) ===
	;; 0x5555 - 0x1111 = 0x4444
	ldd	#0x5555
	subd	#0x1111
	tfr	a,b
	tfr	b,a
	jsr	puthex
	ldd	#0x5555
	subd	#0x1111
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 4444

	rts
