; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-ptr.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Codegen execution test: pointer dereference, array indexing, struct access.

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

;;; Test data (big-endian byte pairs — .word is little-endian, bug in assembler)
arr_i16:
	.byte	0x11, 0x11	; 0x1111
	.byte	0x22, 0x22	; 0x2222
	.byte	0x33, 0x33	; 0x3333
	.byte	0x44, 0x44	; 0x4444
	.byte	0x55, 0x55	; 0x5555

arr_i8:
	.byte	0xAA, 0xBB, 0xCC, 0xDD, 0xEE

;;; Struct-like data: 2 bytes + 2 bytes + 2 bytes (field at offset 4)
struct_data:
	.byte	0x00, 0x01	; field0 (offset 0) = 0x0001
	.byte	0x00, 0x02	; field1 (offset 2) = 0x0002
	.byte	0xBE, 0xEF	; field2 (offset 4) = 0xBEEF

	.section .bss
buf:	.space	10		; writable buffer

	.section .rom,"ax",@progbits

	.globl	test_main
test_main:

	;; ===== Pointer dereference =====

	;; deref(&arr_i16[0]) = 0x1111
	ldx	#arr_i16
	jsr	deref
	jsr	putx
	jsr	putnl
; CHECK: 1111

	;; ===== Array indexing (i16) =====

	;; array_get(arr_i16, 0) = 0x1111
	ldd	#0		; i = 0
	pshs	d
	ldx	#arr_i16
	jsr	array_get
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1111

	;; array_get(arr_i16, 2) = 0x3333
	ldd	#2		; i = 2
	pshs	d
	ldx	#arr_i16
	jsr	array_get
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 3333

	;; array_get(arr_i16, 4) = 0x5555
	ldd	#4		; i = 4
	pshs	d
	ldx	#arr_i16
	jsr	array_get
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 5555

	;; ===== Byte array indexing (i8) =====

	;; byte_get(arr_i8, 0) = 0xAA
	ldd	#0
	pshs	d
	ldx	#arr_i8
	jsr	byte_get
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: AA

	;; byte_get(arr_i8, 3) = 0xDD
	ldd	#3
	pshs	d
	ldx	#arr_i8
	jsr	byte_get
	leas	2,s
	tfr	b,a
	jsr	puthex
	jsr	putnl
; CHECK-NEXT: DD

	;; ===== Pointer write + read back =====

	;; deref_write(buf, 0xCAFE); deref(buf) = 0xCAFE
	ldd	#0xCAFE
	pshs	d
	ldx	#buf
	jsr	deref_write
	leas	2,s
	ldx	#buf
	jsr	deref
	jsr	putx
	jsr	putnl
; CHECK-NEXT: CAFE

	;; ===== Array store + read back =====

	;; array_set(buf, 1, 0xDEAD); array_get(buf, 1) = 0xDEAD
	ldd	#0xDEAD		; val
	pshs	d
	ldd	#1		; i = 1
	pshs	d
	ldx	#buf
	jsr	array_set
	leas	4,s
	;; read back
	ldd	#1
	pshs	d
	ldx	#buf
	jsr	array_get
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: DEAD

	;; ===== Struct field access =====

	;; struct_field(struct_data) = 0xBEEF (offset 4)
	ldx	#struct_data
	jsr	struct_field
	jsr	putx
	jsr	putnl
; CHECK-NEXT: BEEF

	jsr	halt
