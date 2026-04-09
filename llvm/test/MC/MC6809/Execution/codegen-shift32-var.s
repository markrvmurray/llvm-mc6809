;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-var.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-var.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-var.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-var.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: i32 variable shifts via codegen → __ashlsi3.

.include "runtime.inc"
	.section .rom,"ax",@progbits

;;; Only __ashlsi3 to save ROM space
	.globl	__ashlsi3
	.type	__ashlsi3,@function
__ashlsi3:
	pshs	y
	leas	-4,s
	stx	,s
	lda	8,s
	sta	2,s
	lda	9,s
	sta	3,s
	tstb
	beq	.Lshl32_done
.Lshl32_loop:
	asl	3,s
	rol	2,s
	rol	1,s
	rol	,s
	decb
	bne	.Lshl32_loop
.Lshl32_done:
	ldx	,s
	ldd	2,s
	leas	4,s
	puls	y
	std	2,s
	rts

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

	;; shl32(1, 16) = 0x00010000
	;; Use X to push a_lo (pshs x), then set B, to avoid clobbering B.
	ldx	#1		; a_lo
	pshs	x		; push a_lo without touching B
	ldb	#16		; shift count
	ldx	#0		; a_hi
	jsr	shl32
	;; X = result_hi, [S] = result_lo
	ldd	,s
	leas	2,s
	pshs	d		; save lo (putx clobbers D)
	jsr	putx		; print hi
	puls	d
	tfr	d,x
	jsr	putx		; print lo
	jsr	putnl
; CHECK: 00010000

	;; shl32(0xDEAD, 0, 8) = 0x00AD0000 ... wait, shl32(0x00DEAD, 8)
	;; Actually: shl32(0x0000DEAD, 8) = 0x00DEAD00
	ldx	#0xDEAD		; a_lo
	pshs	x		; push a_lo without touching B
	ldb	#8		; shift count
	ldx	#0		; a_hi
	jsr	shl32
	ldd	,s
	leas	2,s
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00DEAD00

	jsr	halt
