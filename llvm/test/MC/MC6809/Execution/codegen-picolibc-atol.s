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
; KNOWN-FAIL: this test exists to keep bug #62 visible.
; test_atol crashes the compiler at every opt level with "Unexpected
; physical register copy" in copyPhysReg, because the optimizer
; reduces `sign` to a 1-bit value, the regalloc spills it to a
; SPILL_*LSB pseudo, and copyPhysReg has no case for the resulting
; `BIT1 ← SPILL_*LSB` copy. The llc step itself fails (no .s output
; produced), so the test fails before it can run any tokens through
; usim. Once bug #62 is fixed, this whole file can move into
; codegen-picolibc.s.

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
