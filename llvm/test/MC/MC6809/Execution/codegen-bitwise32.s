;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-bitwise32.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-bitwise32.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-bitwise32.ll -o %t-raw.s 2>/dev/null
; RUN: sed 's/bsr/lbsr/g' %t-raw.s | grep -v '\.directpage' > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: 32-bit bitwise AND, OR, XOR.

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

;;; op32_and_print — call i32 binary op and print result
;;; Stack: [ret:2][a_hi:2][a_lo:2][b_hi:2][b_lo:2]
;;; Calls via generated function name (appended after this file)
op32_and_print:
	;; Load args for i32 CC: X=a_hi, stack=a_lo,b_hi,b_lo
	leas	-6,s
	ldd	12,s		; a_lo
	std	,s
	ldd	14,s		; b_hi
	std	2,s
	ldd	16,s		; b_lo
	std	4,s
	ldx	10,s		; X = a_hi
	jsr	[8,s]		; indirect call
	;; X=result_hi, ,s=result_lo
	jsr	putx		; print hi (X)
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx		; print lo
	jsr	putnl
	leas	6,s
	rts

	.globl	test_main
test_main:

	;; and32(0x12345678, 0xFF00FF00) = 0x12005600
	ldd	#0xFF00
	pshs	d
	ldd	#0xFF00
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#test_and32
	pshs	d
	jsr	op32_and_print
	leas	10,s
; CHECK: 12005600

	;; or32(0x12005600, 0x00340078) = 0x12345678
	ldd	#0x0078
	pshs	d
	ldd	#0x0034
	pshs	d
	ldd	#0x5600
	pshs	d
	ldd	#0x1200
	pshs	d
	ldd	#test_or32
	pshs	d
	jsr	op32_and_print
	leas	10,s
; CHECK-NEXT: 12345678

	;; xor32(0xFFFFFFFF, 0xAAAAAAAA) = 0x55555555
	ldd	#0xAAAA
	pshs	d
	ldd	#0xAAAA
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#0xFFFF
	pshs	d
	ldd	#test_xor32
	pshs	d
	jsr	op32_and_print
	leas	10,s
; CHECK-NEXT: 55555555

	;; xor32(0x12345678, 0x12345678) = 0x00000000 (self-XOR)
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#0x5678
	pshs	d
	ldd	#0x1234
	pshs	d
	ldd	#test_xor32
	pshs	d
	jsr	op32_and_print
	leas	10,s
; CHECK-NEXT: 00000000

	jsr	halt
