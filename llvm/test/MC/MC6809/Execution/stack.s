; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=200000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Stack operation execution tests.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; === PSHS/PULS single register ===
	lda	#0x42
	pshs	a
	lda	#0x00		; clobber A
	puls	a		; restore
	jsr	puthex
	jsr	putnl
; CHECK: 42

	;; === PSHS/PULS B ===
	ldb	#0xAB
	pshs	b
	ldb	#0x00
	puls	b
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: AB

	;; === PSHS/PULS multiple registers ===
	;; Push A,B then pull them back
	lda	#0x12
	ldb	#0x34
	pshs	b,a		; push A then B (A at lower addr)
	lda	#0x00
	ldb	#0x00
	puls	b,a		; pull B then A
	jsr	puthex		; print A = 0x12
	tfr	b,a
	jsr	puthex		; print B = 0x34
	jsr	putnl
; CHECK-NEXT: 1234

	;; === PSHS/PULS X (16-bit register) ===
	ldx	#0xBEEF
	pshs	x
	ldx	#0x0000
	puls	x
	;; Print X high byte and low byte
	tfr	x,d		; D = X (A=high, B=low)
	pshs	b		; save low
	jsr	puthex		; print high
	puls	a		; restore low into A
	jsr	puthex		; print low
	jsr	putnl
; CHECK-NEXT: BEEF

	;; === PSHS/PULS CC (condition codes) ===
	;; Set carry, save CC, clear carry, restore CC, test carry
	orcc	#0x01		; set carry
	pshs	cc
	andcc	#0xfe		; clear carry
	puls	cc		; restore (carry should be back)
	bcs	.Lcc1_yes
	lda	#0x4e		; 'N'
	bra	.Lcc1_out
.Lcc1_yes:
	lda	#0x59		; 'Y'
.Lcc1_out:
	jsr	putchar
	jsr	putnl
; CHECK-NEXT: Y

	;; === JSR/RTS nesting depth ===
	;; Call 3 levels deep, each adds to a value in B
	ldb	#0x00
	jsr	nest1		; nest1 calls nest2 calls nest3
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 07

	;; === PSHS/PULS with U register ===
	;; Verify U survives push/pull cycle
	pshs	u
	ldu	#0x1234
	pshs	u
	ldu	#0x0000
	puls	u
	;; U should be 0x1234
	pshs	x		; save X
	tfr	u,x
	tfr	x,d
	pshs	b
	jsr	puthex		; high byte
	puls	a
	jsr	puthex		; low byte
	puls	x		; restore X
	puls	u		; restore original U
	jsr	putnl
; CHECK-NEXT: 1234

	;; === LEA for stack frame manipulation ===
	;; LEAS -4,S allocates 4 bytes, LEAS 4,S frees them
	lda	#0xAA
	leas	-4,s		; allocate
	sta	,s		; store at top of allocated space
	lda	#0x00		; clobber
	lda	,s		; read back
	leas	4,s		; free
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: AA

	;; === PSHU/PULU (user stack) ===
	;; Save/restore S register, use U as alternate stack
	pshs	u		; save original U
	ldu	#0x0200		; set U stack at 0x0200
	lda	#0x55
	pshu	a		; push onto U stack
	lda	#0x00
	pulu	a		; pull from U stack
	puls	u		; restore original U
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 55

	;; === Stack-relative addressing ===
	;; Push several values, access them by offset
	lda	#0x11
	pshs	a
	lda	#0x22
	pshs	a
	lda	#0x33
	pshs	a
	;; Stack: S+0=0x33, S+1=0x22, S+2=0x11
	lda	2,s		; read 0x11
	jsr	puthex
	lda	1,s		; read 0x22
	jsr	puthex
	lda	,s		; read 0x33
	jsr	puthex
	leas	3,s		; clean up
	jsr	putnl
; CHECK-NEXT: 112233

	rts

;; Nested functions for JSR/RTS depth test
nest1:
	addb	#1		; B += 1
	jsr	nest2
	rts
nest2:
	addb	#2		; B += 2
	jsr	nest3
	rts
nest3:
	addb	#4		; B += 4
	rts
