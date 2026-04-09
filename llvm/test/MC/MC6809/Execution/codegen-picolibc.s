;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Picolibc execution tests: memcpy, memset, memcmp, strcpy, strchr,
; strncmp, isspace, isprint, isxdigit, tolower, toupper.
; Tested at -O0 through -O3.

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
	;; memcpy: copy "Hello" (5 bytes) into buf
	ldx	#buf
	leas	-4,s
	ldd	#str_hello
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memcpy
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	ldb	buf+4
	jsr	putb
	jsr	putnl
; CHECK: 48656C6C6F

	;; memset: fill 4 bytes with 0xAA
	ldx	#buf
	leas	-4,s
	ldd	#0xAA
	std	,s
	ldd	#4
	std	2,s
	jsr	test_memset
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	jsr	putnl
; CHECK-NEXT: AAAAAAAA

	;; memmove: forward (dst < src) — copy "Hello" forward into buf
	ldx	#buf
	leas	-4,s
	ldd	#str_hello
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memmove
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	ldb	buf+4
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 48656C6C6F

	;; memmove: backward (dst > src, overlapping) — shift buf right by 1
	;; buf currently = "Hello"
	;; Move buf+0..4 → buf+1..5 (overlap)
	ldx	#buf+1
	leas	-4,s
	ldd	#buf
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memmove
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	ldb	buf+4
	jsr	putb
	ldb	buf+5
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 4848656C6C6F

	;; strcpy: copy "Hi" into buf
	ldx	#buf
	leas	-2,s
	ldd	#str_hi
	std	,s
	jsr	test_strcpy
	leas	2,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 486900

	;; isspace
	ldx	#0x20
	jsr	test_isspace
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	ldx	#0x41
	jsr	test_isspace
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	ldx	#0x09
	jsr	test_isspace
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isprint
	ldx	#0x41
	jsr	test_isprint
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	ldx	#0x01
	jsr	test_isprint
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; toupper
	ldx	#0x61
	jsr	test_toupper
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0041

	ldx	#0x5A
	jsr	test_toupper
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 005A

	;; memcmp: "Hello" vs "Hello" = 0
	ldx	#str_hello
	leas	-4,s
	ldd	#str_hello
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memcmp
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; memcmp: "Hello" vs "Help!" (diff at 'l' vs 'p' = -4)
	ldx	#str_hello
	leas	-4,s
	ldd	#str_help
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memcmp
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFC

	;; isxdigit('9') = 1
	ldx	#0x39
	jsr	test_isxdigit
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isxdigit('g') = 0
	ldx	#0x67
	jsr	test_isxdigit
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; tolower('A') = 'a' = 0x61
	ldx	#0x41
	jsr	test_tolower
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0061

	;; tolower('z') = 'z' = 0x7A (unchanged)
	ldx	#0x7A
	jsr	test_tolower
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 007A

	;; strchr: find 'l' in "Hello" → pointer to 3rd char
	ldx	#str_hello
	leas	-2,s
	ldd	#0x6C		; 'l'
	std	,s
	jsr	test_strchr
	leas	2,s
	;; Result is a pointer — compute offset from str_hello
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; strchr: find 'z' in "Hello" → NULL (0)
	ldx	#str_hello
	leas	-2,s
	ldd	#0x7A		; 'z'
	std	,s
	jsr	test_strchr
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strncmp: "Hello" vs "Help!", 3 chars = 0 (first 3 match)
	ldx	#str_hello
	leas	-4,s
	ldd	#str_help
	std	,s
	ldd	#3
	std	2,s
	jsr	test_strncmp
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strncmp: "Hello" vs "Help!", 5 chars = diff at 'l' vs 'p'
	ldx	#str_hello
	leas	-4,s
	ldd	#str_help
	std	,s
	ldd	#5
	std	2,s
	jsr	test_strncmp
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: FFFC

	;; strlen("Hello") = 5
	ldx	#str_hello
	jsr	test_strlen
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0005

	;; strrchr("Hello", 'l') → offset 3 (last 'l')
	ldx	#str_hello
	leas	-2,s
	ldd	#0x6C
	std	,s
	jsr	test_strrchr
	leas	2,s
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0003

	;; strrchr("Hello", 'z') → NULL
	ldx	#str_hello
	leas	-2,s
	ldd	#0x7A
	std	,s
	jsr	test_strrchr
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strstr("Hello", "ll") → offset 2
	;; KNOWN BUG #58: test_strstr corrupts the stack at -O0; works at -O1+.
	;; Left in deliberately so the failure stays visible.
	ldx	#str_hello
	leas	-2,s
	ldd	#str_ll
	std	,s
	jsr	test_strstr
	leas	2,s
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; strstr("Hello", "xy") → NULL
	ldx	#str_hello
	leas	-2,s
	ldd	#str_xy
	std	,s
	jsr	test_strstr
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strstr("Hello", "") → "Hello" (offset 0)
	ldx	#str_hello
	leas	-2,s
	ldd	#str_empty
	std	,s
	jsr	test_strstr
	leas	2,s
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; memchr("Hello", 'l', 5) → offset 2
	ldx	#str_hello
	leas	-4,s
	ldd	#0x6C
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memchr
	leas	4,s
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; memchr("Hello", 'z', 5) → NULL
	ldx	#str_hello
	leas	-4,s
	ldd	#0x7A
	std	,s
	ldd	#5
	std	2,s
	jsr	test_memchr
	leas	4,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strspn("Hello", "elH") = 4 (H,e,l,l all match; o doesn't)
	ldx	#str_hello
	leas	-2,s
	ldd	#str_elH
	std	,s
	jsr	test_strspn
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0004

	;; strcspn("Hello", "lo") = 2 (H,e don't match; l matches)
	ldx	#str_hello
	leas	-2,s
	ldd	#str_lo
	std	,s
	jsr	test_strcspn
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; strpbrk("Hello", "lo") → "llo" (offset 2)
	ldx	#str_hello
	leas	-2,s
	ldd	#str_lo
	std	,s
	jsr	test_strpbrk
	leas	2,s
	tfr	x,d
	subd	#str_hello
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; strncpy(buf, "Hi", 4) → "Hi\0\0"
	ldx	#buf
	leas	-4,s
	ldd	#str_hi
	std	,s
	ldd	#4
	std	2,s
	jsr	test_strncpy
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 48690000

	;; strcpy(buf, "He"); strcat(buf, "y!") → "Hey!\0"
	ldx	#buf
	leas	-2,s
	ldd	#str_He
	std	,s
	jsr	test_strcpy
	leas	2,s
	ldx	#buf
	leas	-2,s
	ldd	#str_yex
	std	,s
	jsr	test_strcat
	leas	2,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	ldb	buf+4
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 4865792100

	;; strcpy(buf, "Hi"); strncat(buf, "ho!", 2) → "Hiho\0"
	ldx	#buf
	leas	-2,s
	ldd	#str_hi
	std	,s
	jsr	test_strcpy
	leas	2,s
	ldx	#buf
	leas	-4,s
	ldd	#str_hox
	std	,s
	ldd	#2
	std	2,s
	jsr	test_strncat
	leas	4,s
	ldb	buf
	jsr	putb
	ldb	buf+1
	jsr	putb
	ldb	buf+2
	jsr	putb
	ldb	buf+3
	jsr	putb
	ldb	buf+4
	jsr	putb
	jsr	putnl
; CHECK-NEXT: 4869686F00

	rts

	.section .rodata,"a",@progbits
str_hello:	.asciz	"Hello"
str_hi:		.asciz	"Hi"
str_help:	.asciz	"Help!"
str_ll:		.asciz	"ll"
str_xy:		.asciz	"xy"
str_empty:	.asciz	""
str_elH:	.asciz	"elH"
str_lo:		.asciz	"lo"
str_He:		.asciz	"He"
str_yex:	.asciz	"y!"
str_hox:	.asciz	"ho!"

	.section .bss,"aw",@nobits
buf:	.space	16
