;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-atol.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-atol.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-atol.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc-atol.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; KNOWN-FAIL: this test exists to keep bug #57 visible.
;
; History: this file originally caught bug #62 (compile crash in
; copyPhysReg from SPILL_*LSB) — that bug is now fixed. atol now
; compiles cleanly at every opt level, but the runtime exposes bug
; #57 (BIT1↔CC linkage phantom).
;
; Symptom: atol("42") returns 0x000A002A instead of 0x0000002A. The
; lo half (42) is correct, the hi half (10 = the multiplier) leaks
; in. atol("0") and atol("-3") work correctly.
;
; Root cause: in the loop body, `result = result * 10 + (*s - '0')`,
; the codegen interleaves the next iteration's `(*s - '0')` (an
; `addb #-48`) BETWEEN this iteration's `addb 15,u; adda 14,u` (the
; lo half of the i32 add, sets CC.C) and `adcb #0; adca #0` (the hi
; half, propagates CC.C). The intervening addb #-48 clobbers CC.C.
; The carry should flow through a BIT1 vreg from the lo add to the
; hi propagation, but the BIT1 vreg lives in AALSB/ABLSB which is a
; phantom for CC.C — see bug #57 in the tracker for the full design
; discussion. Once #57 has a working fix, this whole file can move
; into codegen-picolibc.s.

.include "runtime.inc"
.include "mc6809rt.s"

	.section .rom,"ax",@progbits

putx:
	pshs	x,d
	tfr	x,d
	tfr	a,b
	tfr	b,a
	jsr	puthex
	puls	x,d
	pshs	x,d
	tfr	x,d
	tfr	b,a
	jsr	puthex
	puls	x,d
	rts

	.globl	test_main
test_main:
	;; atol("42") = 42  (= 0x0000002A)
	ldx	#str_42
	leas	-2,s
	jsr	test_atol
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK: 0000002A

	;; atol("-3") = -3  (= 0xFFFFFFFD)
	ldx	#str_neg3
	leas	-2,s
	jsr	test_atol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: FFFFFFFD

	;; atol("0") = 0
	ldx	#str_0
	leas	-2,s
	jsr	test_atol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000000

	rts

	.section .rodata,"a",@progbits
str_42:		.asciz	"42"
str_neg3:	.asciz	"-3"
str_0:		.asciz	"0"
