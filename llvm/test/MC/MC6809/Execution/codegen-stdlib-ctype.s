; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Execution tests for stdlib (abs, atoi, div) and ctype (isdigit, isalpha,
; isupper, islower) functions compiled from C. Tested at -O0 through -O3.

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

putb:
	tfr	b,a
	jsr	puthex
	rts

	.globl	test_main
test_main:
	;; ===== abs =====
	ldx	#5
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK: 0005

	ldx	#-3
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0003

	ldx	#0
	jsr	test_abs
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ===== atoi (positive, bug #52 fix) =====
	ldx	#str_42
	jsr	test_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 002A

	ldx	#str_0
	jsr	test_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	ldx	#str_999
	jsr	test_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 03E7

	;; ===== atoi_neg (positive and negative — bug #49 fix) =====
	ldx	#str_42
	jsr	test_atoi_neg
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 002A

	ldx	#str_neg3
	jsr	test_atoi_neg
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFD

	ldx	#str_0
	jsr	test_atoi_neg
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	ldx	#str_neg1
	jsr	test_atoi_neg
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	ldx	#str_neg999
	jsr	test_atoi_neg
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FC19

	;; ===== div (via test_div: result[0]=quot, result[1]=rem) =====
	;; 17 / 5 = 3 rem 2
	ldx	#divbuf
	leas	-4,s
	ldd	#17
	std	,s
	ldd	#5
	std	2,s
	jsr	test_div
	leas	4,s
	ldd	divbuf		; quot
	tfr	d,x
	jsr	putx
	ldd	divbuf+2	; rem
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 00030002

	;; ===== 16-bit bitwise =====
	ldx	#0x00FF
	leas	-2,s
	ldd	#0xFF00
	std	,s
	jsr	test_or16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFF

	ldx	#0x0F0F
	leas	-2,s
	ldd	#0xFF00
	std	,s
	jsr	test_and16
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0F00

	;; ===== ctype =====

	;; isdigit('5') = 1
	ldx	#0x35
	jsr	test_isdigit
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isdigit('A') = 0
	ldx	#0x41
	jsr	test_isdigit
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isdigit(' ') = 0  (early-exit false path)
	ldx	#0x20
	jsr	test_isdigit
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isalpha('a') = 1
	ldx	#0x61
	jsr	test_isalpha
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isalpha('3') = 0  (early-exit false path)
	ldx	#0x33
	jsr	test_isalpha
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isupper('A') = 1
	ldx	#0x41
	jsr	test_isupper
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isupper('a') = 0
	ldx	#0x61
	jsr	test_isupper
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isupper('!') = 0  (early-exit false path)
	ldx	#0x21
	jsr	test_isupper
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; islower('z') = 1
	ldx	#0x7A
	jsr	test_islower
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; islower('Z') = 0  (early-exit false path)
	ldx	#0x5A
	jsr	test_islower
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	rts

	.section .rodata,"a",@progbits
str_42:		.asciz	"42"
str_0:		.asciz	"0"
str_999:	.asciz	"999"
str_neg3:	.asciz	"-3"
str_neg1:	.asciz	"-1"
str_neg999:	.asciz	"-999"

	.section .data,"aw",@progbits
divbuf:		.space	4, 0
