; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj %s -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=200000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Addressing mode execution tests.
; Uses RAM at 0x0100-0x010F as scratch space.

.include "runtime.inc"

	.section .rom,"ax",@progbits
	.globl	test_main
test_main:
	;; Set up test data in RAM
	lda	#0x11
	sta	0x0100
	lda	#0x22
	sta	0x0101
	lda	#0x33
	sta	0x0102
	lda	#0x44
	sta	0x0103
	lda	#0x55
	sta	0x0104

	;; === Extended addressing (16-bit absolute) ===
	lda	0x0100
	jsr	puthex
	jsr	putnl
; CHECK: 11

	;; === Direct page addressing ===
	;; Set DP = 0x01, then access <0x00 = 0x0100
	lda	#0x01
	tfr	a,dp
	lda	<0x02		; reads 0x0102 = 0x33
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 33

	;; Reset DP to 0 for remaining tests
	clra
	tfr	a,dp

	;; === Indexed: zero offset ===
	ldx	#0x0100
	lda	,x		; [X+0] = 0x11
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 11

	;; === Indexed: 5-bit offset ===
	lda	3,x		; [X+3] = 0x44
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 44

	;; === Indexed: negative 5-bit offset ===
	ldx	#0x0104
	lda	-2,x		; [0x0104-2] = [0x0102] = 0x33
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 33

	;; === Indexed: 8-bit offset ===
	ldx	#0x0100
	lda	4,x		; [0x0100+4] = 0x55
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 55

	;; === Indexed: Y register ===
	ldy	#0x0101
	lda	,y		; [Y+0] = 0x22
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 22

	;; === Indexed: U register ===
	;; Save and restore U around this test
	pshs	u
	ldu	#0x0103
	lda	,u		; [U+0] = 0x44
	puls	u
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 44

	;; === Indexed: S register ===
	;; Push a known value, read it via S offset
	lda	#0xAB
	pshs	a		; push 0xAB onto stack
	lda	,s		; [S+0] = 0xAB
	leas	1,s		; discard
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: AB

	;; === Auto-increment: ,X+ ===
	ldx	#0x0100
	lda	,x+		; A = [0x0100] = 0x11, X becomes 0x0101
	jsr	puthex
	lda	,x+		; A = [0x0101] = 0x22, X becomes 0x0102
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 1122

	;; === Auto-increment: ,X++ (word increment) ===
	ldx	#0x0100
	lda	,x++		; A = [0x0100] = 0x11, X becomes 0x0102
	jsr	puthex
	lda	,x++		; A = [0x0102] = 0x33, X becomes 0x0104
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 1133

	;; === Auto-decrement: ,-X ===
	ldx	#0x0105
	lda	,-x		; X becomes 0x0104, A = [0x0104] = 0x55
	jsr	puthex
	lda	,-x		; X becomes 0x0103, A = [0x0103] = 0x44
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 5544

	;; === Auto-decrement: ,--X (word decrement) ===
	ldx	#0x0105
	lda	,--x		; X becomes 0x0103, A = [0x0103] = 0x44
	jsr	puthex
	lda	,--x		; X becomes 0x0101, A = [0x0101] = 0x22
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 4422

	;; === Accumulator offset: A,X ===
	ldx	#0x0100
	lda	#0x03
	lda	a,x		; [X + A] = [0x0100+3] = 0x44
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 44

	;; === Accumulator offset: B,X ===
	ldx	#0x0100
	ldb	#0x04
	lda	b,x		; [X + B] = [0x0100+4] = 0x55
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 55

	;; === Accumulator offset: D,X ===
	ldx	#0x0100
	ldd	#0x0002
	lda	d,x		; [X + D] = [0x0100+2] = 0x33
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 33

	;; === PC-relative: n,PC ===
	lda	pcdata,pc	; load from PC-relative address
	jsr	puthex
	jsr	putnl
	bra	after_pcdata
pcdata:
	.byte	0x42
after_pcdata:
; CHECK-NEXT: 42

	;; === Indirect: [n,X] ===
	;; Store pointer 0x0103 at 0x0108-0x0109
	ldd	#0x0103
	std	0x0108
	ldx	#0x0108
	lda	[,x]		; read pointer at [X], then load from that addr
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 44

	rts
