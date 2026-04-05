;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: conditionals (icmp + select) for i8, i16, and i32.

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

	;; ===== Signed max i8 (sgt) =====

	;; max_s8(5, 3) = 5
	ldd	#3
	pshs	d
	ldb	#5
	jsr	test_max_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK: 05

	;; max_s8(3, 5) = 5
	ldd	#5
	pshs	d
	ldb	#3
	jsr	test_max_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 05

	;; max_s8(-1, 1) = 1  (0xFF vs 0x01 signed)
	ldd	#1
	pshs	d
	ldb	#0xFF
	jsr	test_max_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	;; max_s8(7, 7) = 7  (equal)
	ldd	#7
	pshs	d
	ldb	#7
	jsr	test_max_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 07

	;; ===== Signed max i16 (sgt) =====

	;; max_s16(1000, 500) = 1000 = 0x03E8
	ldd	#500
	pshs	d
	ldx	#1000
	jsr	test_max_s16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 03E8

	;; max_s16(-1, 1) = 1 (0xFFFF vs 0x0001)
	ldd	#1
	pshs	d
	ldx	#0xFFFF
	jsr	test_max_s16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; ===== Unsigned max i8 (ugt) =====

	;; max_u8(0xFF, 0x01) = 0xFF (unsigned: 255 > 1)
	ldd	#1
	pshs	d
	ldb	#0xFF
	jsr	test_max_u8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; ===== Unsigned max i16 (ugt) =====

	;; max_u16(0xFFFF, 0x0001) = 0xFFFF
	ldd	#1
	pshs	d
	ldx	#0xFFFF
	jsr	test_max_u16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	;; ===== Signed min i8 (slt) =====

	;; min_s8(5, 3) = 3
	ldd	#3
	pshs	d
	ldb	#5
	jsr	test_min_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 03

	;; min_s8(-1, 1) = -1 = 0xFF
	ldd	#1
	pshs	d
	ldb	#0xFF
	jsr	test_min_s8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: FF

	;; ===== Equality i8 (eq) =====

	;; eq8(42, 42) = 1 (equal)
	ldd	#42
	pshs	d
	ldb	#42
	jsr	test_eq8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	;; eq8(42, 43) = 0 (not equal)
	ldd	#43
	pshs	d
	ldb	#42
	jsr	test_eq8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; ===== Not-equal i8 (ne) =====

	;; ne8(42, 43) = 1 (not equal)
	ldd	#43
	pshs	d
	ldb	#42
	jsr	test_ne8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 01

	;; ne8(42, 42) = 0 (equal)
	ldd	#42
	pshs	d
	ldb	#42
	jsr	test_ne8
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: 00

	;; ===== Equality i16 (eq) =====

	;; eq16(0x1234, 0x1234) = 1
	ldd	#0x1234
	pshs	d
	ldx	#0x1234
	jsr	test_eq16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; eq16(0x1234, 0x5678) = 0
	ldd	#0x5678
	pshs	d
	ldx	#0x1234
	jsr	test_eq16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ===== Not-equal i16 (ne) =====

	;; ne16(0x1234, 0x5678) = 1
	ldd	#0x5678
	pshs	d
	ldx	#0x1234
	jsr	test_ne16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; ne16(0xAAAA, 0xAAAA) = 0
	ldd	#0xAAAA
	pshs	d
	ldx	#0xAAAA
	jsr	test_ne16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ===== i32 signed max (sgt + select) =====
	;; Tests bug #30 fix: CMPX used for hi-word to avoid D clobber

	;; max_s32(3, 1) = 3
	;; a = 0x00000003, b = 0x00000001
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#3		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	test_max_s32
	;; X = result_hi, [S] = result_lo
	ldd	,s		; D = result_lo
	leas	6,s		; clean 3 words
	pshs	d		; save result_lo (putx clobbers D)
	jsr	putx		; print hi word (X)
	puls	d		; restore result_lo
	tfr	d,x
	jsr	putx		; print lo word
	jsr	putnl
; CHECK-NEXT: 00000003

	;; max_s32(1, 3) = 3
	ldd	#3		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#1		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	test_max_s32
	ldd	,s
	leas	6,s
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00000003

	;; max_s32(-1, 1) = 1  (0xFFFFFFFF vs 0x00000001)
	ldd	#1		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0xFFFF		; a_lo
	pshs	d
	ldx	#0xFFFF		; a_hi
	jsr	test_max_s32
	ldd	,s
	leas	6,s
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00000001

	;; max_s32(0x12345678, 0x12345679) = 0x12345679
	;; Same hi, lo differs: tests the lo-word compare path
	ldd	#0x5679		; b_lo
	pshs	d
	ldd	#0x1234		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_max_s32
	ldd	,s
	leas	6,s
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 12345679

	;; ===== i32 unsigned min (ult + select) =====

	;; min_u32(5, 3) = 3
	ldd	#3		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#5		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	test_min_u32
	ldd	,s
	leas	6,s
	pshs	d
	jsr	putx
	puls	d
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00000003

	;; ===== i32 equality (eq) =====

	;; eq32(0x12345678, 0x12345678) = 1 (equal)
	ldd	#0x5678		; b_lo
	pshs	d
	ldd	#0x1234		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_eq32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; eq32(0x12345678, 0x12345679) = 0 (lo differs)
	ldd	#0x5679		; b_lo
	pshs	d
	ldd	#0x1234		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_eq32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; eq32(0x12345678, 0x12345678) but hi differs = 0
	ldd	#0x5678		; b_lo
	pshs	d
	ldd	#0x1235		; b_hi (differs)
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_eq32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; eq32(0, 0) = 1
	ldd	#0		; b_lo
	pshs	d
	ldd	#0		; b_hi
	pshs	d
	ldd	#0		; a_lo
	pshs	d
	ldx	#0		; a_hi
	jsr	test_eq32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; ===== i32 not-equal (ne) =====

	;; ne32(0x12345678, 0x12345678) = 0 (equal)
	ldd	#0x5678		; b_lo
	pshs	d
	ldd	#0x1234		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_ne32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ne32(0x12345678, 0xFFFFFFFF) = 1 (totally different)
	ldd	#0xFFFF		; b_lo
	pshs	d
	ldd	#0xFFFF		; b_hi
	pshs	d
	ldd	#0x5678		; a_lo
	pshs	d
	ldx	#0x1234		; a_hi
	jsr	test_ne32
	leas	6,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	jsr	halt
