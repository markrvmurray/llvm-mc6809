; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-libc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Libc smoke test: strlen, strcmp, atoi.

.include "runtime.inc"
.include "mc6809rt.s"

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

;;; Test data
src_hello:
	.byte	0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x00	; "Hello\0"
src_hell:
	.byte	0x48, 0x65, 0x6C, 0x6C, 0x00		; "Hell\0"
src_help:
	.byte	0x48, 0x65, 0x6C, 0x70, 0x00		; "Help\0"
src_123:
	.byte	0x31, 0x32, 0x33, 0x00			; "123\0"
src_9999:
	.byte	0x39, 0x39, 0x39, 0x39, 0x00		; "9999\0"
src_empty:
	.byte	0x00					; "\0"

	.section .bss
buf:	.space	16

	.section .rom,"ax",@progbits

	.globl	test_main
test_main:

	;; ===== strlen =====

	;; strlen("Hello") = 5
	ldx	#src_hello
	jsr	my_strlen
	jsr	putx
	jsr	putnl
; CHECK: 0005

	;; strlen("") = 0
	ldx	#src_empty
	jsr	my_strlen
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; NOTE: memcpy/memset skipped — register pressure bugs (#38).

	;; ===== strcmp =====

	;; strcmp("Hello", "Hello") = 0
	ldd	#src_hello	; b
	pshs	d
	ldx	#src_hello	; a
	jsr	my_strcmp
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strcmp("Hello", "Hell") > 0  ('o' - '\0' = 111 = 0x006F)
	ldd	#src_hell
	pshs	d
	ldx	#src_hello
	jsr	my_strcmp
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 006F

	;; strcmp("Hell", "Help") < 0  ('l' - 'p' = -4 = 0xFFFC)
	ldd	#src_help
	pshs	d
	ldx	#src_hell
	jsr	my_strcmp
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFC

	;; ===== atoi =====

	;; atoi("123") = 123 = 0x007B
	ldx	#src_123
	jsr	my_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 007B

	;; atoi("9999") = 9999 = 0x270F
	ldx	#src_9999
	jsr	my_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 270F

	;; atoi("") = 0
	ldx	#src_empty
	jsr	my_atoi
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	jsr	halt
