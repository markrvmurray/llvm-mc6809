;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: cat %s %t-funcs.s | llvm-mc -triple=mc6809 -I %S/Inputs \
; RUN:   --filetype=obj -o %t.o
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Picolibc execution tests: 34 functions covering string.h (memcpy,
; memset, memmove, memcmp, memchr, strcpy, strncpy, strcat, strncat,
; strlen, strchr, strrchr, strstr, strspn, strcspn, strpbrk, strncmp,
; strtok), ctype.h (isspace, isprint, isxdigit, tolower, toupper,
; ispunct, iscntrl, isgraph), stdlib.h (labs, atol, strtol, strtoul,
; rand, srand), and stdio.h (putchar, printf — varargs torture test).
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

	;; strtok test: copy "a,b,c\0" into buf, then tokenize on ","
	;; First, memcpy the source string into buf (strtok modifies its input).
	ldx	#buf
	leas	-4,s
	ldd	#str_abc
	std	,s
	ldd	#6
	std	2,s
	jsr	test_memcpy
	leas	4,s

	;; strtok(buf, ",") → "a" at offset 0
	ldx	#buf
	leas	-2,s
	ldd	#str_comma
	std	,s
	jsr	test_strtok
	leas	2,s
	tfr	x,d
	subd	#buf
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; strtok(NULL, ",") → "b" at offset 2
	ldx	#0
	leas	-2,s
	ldd	#str_comma
	std	,s
	jsr	test_strtok
	leas	2,s
	tfr	x,d
	subd	#buf
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0002

	;; strtok(NULL, ",") → "c" at offset 4
	ldx	#0
	leas	-2,s
	ldd	#str_comma
	std	,s
	jsr	test_strtok
	leas	2,s
	tfr	x,d
	subd	#buf
	tfr	d,x
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0004

	;; strtok(NULL, ",") → NULL (no more tokens)
	ldx	#0
	leas	-2,s
	ldd	#str_comma
	std	,s
	jsr	test_strtok
	leas	2,s
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ispunct('!') = 1
	ldx	#0x21
	jsr	test_ispunct
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; ispunct('A') = 0
	ldx	#0x41
	jsr	test_ispunct
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; ispunct(' ') = 0  (space is not punctuation)
	ldx	#0x20
	jsr	test_ispunct
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; iscntrl('\n') = 1
	ldx	#0x0A
	jsr	test_iscntrl
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; iscntrl(0x7F) = 1  (DEL)
	ldx	#0x7F
	jsr	test_iscntrl
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; iscntrl('A') = 0
	ldx	#0x41
	jsr	test_iscntrl
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isgraph('A') = 1
	ldx	#0x41
	jsr	test_isgraph
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0001

	;; isgraph(' ') = 0  (space is not graph)
	ldx	#0x20
	jsr	test_isgraph
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; isgraph(0x7F) = 0  (DEL is not graph)
	ldx	#0x7F
	jsr	test_isgraph
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0000

	;; labs(5) = 5  (i32: hi=0, lo=5)
	leas	-2,s
	ldd	#5		; lo
	std	,s
	ldx	#0		; hi
	jsr	test_labs
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000005

	;; labs(0) = 0
	leas	-2,s
	ldd	#0		; lo
	std	,s
	ldx	#0		; hi
	jsr	test_labs
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000000

	;; labs(-3) = 3  (-3 = 0xFFFFFFFD → hi=0xFFFF, lo=0xFFFD)
	;; This was bug #61 (the labs(-3) case returned -3 unchanged via
	;; the (0 - x) selector path). Now sidestepped via XOR/ADD lowering
	;; in customAbs — see MC6809LegalizerInfo.cpp G_ABS case. The
	;; underlying selectSubO/E operand-swap bug is still latent.
	leas	-2,s
	ldd	#0xFFFD		; lo
	std	,s
	ldx	#0xFFFF		; hi
	jsr	test_labs
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000003

	;; srand(42); rand(); rand(); rand()
	;; LCG: state = state*75 + 17, return state & 0x7FFF
	;; srand(42) → state=42
	ldx	#42
	jsr	test_srand
	;; rand() → state=42*75+17=3167=0x0C5F, return 0x0C5F
	jsr	test_rand
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 0C5F
	;; rand() → state=3167*75+17=237542 mod 65536=40934=0x9FE6,
	;;          return 0x9FE6 & 0x7FFF = 0x1FE6
	jsr	test_rand
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 1FE6
	;; rand() → state=40934*75+17=3070067 mod 65536=55411=0xD873,
	;;          return 0xD873 & 0x7FFF = 0x5873
	jsr	test_rand
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 5873

	;; srand(1); rand() — verify reseeding works
	ldx	#1
	jsr	test_srand
	;; state=1*75+17=92=0x005C
	jsr	test_rand
	jsr	putx
	jsr	putnl
; CHECK-NEXT: 005C

	;; atol("42") = 42  (= 0x0000002A)
	;; This was bug #57 (carry chain disruption in the loop body's
	;; result*10 + digit add). Now sidestepped via hasSideEffects=1
	;; on AddSetCarry/SubSetCarry pseudos — see the bug tracker for
	;; the design discussion.
	ldx	#str_42
	leas	-2,s
	jsr	test_atol
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 0000002A

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
	ldx	#str_atol_0
	leas	-2,s
	jsr	test_atol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	2,s
	jsr	putnl
; CHECK-NEXT: 00000000

	;; strtol/strtoul tests
	;;
	;; Calling convention for `long strtol(const char *nptr, char **endptr, int base)`:
	;;   nptr   → X
	;;   endptr → ,s     (i16, NULL = 0 if not interested)
	;;   base   → 2,s    (i16)
	;; Return long: result_hi = X, result_lo = ,s (clobbers endptr slot)
	;; Caller allocates 4 bytes for the 2 stack args; the result_lo
	;; lands in the first slot when the function returns.

	;; strtol("42", NULL, 10) = 42
	ldx	#str_42
	leas	-4,s
	ldd	#0		; endptr = NULL
	std	,s
	ldd	#10		; base = 10
	std	2,s
	jsr	test_strtol
	jsr	putx		; result_hi
	ldd	,s		; result_lo
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 0000002A

	;; strtol("-3", NULL, 10) = -3 (= 0xFFFFFFFD)
	ldx	#str_neg3
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#10
	std	2,s
	jsr	test_strtol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: FFFFFFFD

	;; strtol("FF", NULL, 16) = 255 (= 0x000000FF)
	ldx	#str_ff
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#16
	std	2,s
	jsr	test_strtol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 000000FF

	;; strtol("0x10", NULL, 0) = 16 (auto-detect hex from prefix)
	ldx	#str_0x10
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#0		; base = 0 → auto-detect
	std	2,s
	jsr	test_strtol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 00000010

	;; strtol("777", NULL, 8) = 511 (= 0x000001FF)
	ldx	#str_777
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#8
	std	2,s
	jsr	test_strtol
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 000001FF

	;; strtoul("12345", NULL, 10) = 12345 (= 0x00003039)
	ldx	#str_12345
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#10
	std	2,s
	jsr	test_strtoul
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 00003039

	;; strtoul("FFFF", NULL, 16) = 65535 (= 0x0000FFFF)
	ldx	#str_ffff
	leas	-4,s
	ldd	#0
	std	,s
	ldd	#16
	std	2,s
	jsr	test_strtoul
	jsr	putx
	ldd	,s
	tfr	d,x
	jsr	putx
	leas	4,s
	jsr	putnl
; CHECK-NEXT: 0000FFFF

	;; test_putchar — direct ACIA write via volatile pointer.
	;; Picolibc's putchar() is one line; this is the same idea.
	;; putchar('!') = '!' returned (= 0x21)
	ldx	#0x21		; '!'
	jsr	test_putchar
	jsr	putnl
; CHECK-NEXT: !

	;; test_printf — minimal varargs printf clone (the first test
	;; that exercises G_VASTART/G_VAARG end-to-end through codegen).
	;; printf("Hi=%d %s %x %c %%\n", 42, "ok", 0xBEEF, 'Z')
	;;   produces: "Hi=42 ok beef Z %\n"
	;; Calling convention for varargs:
	;;   fmt → X (first non-vararg)
	;;   varargs → stack starting at ,s, in declaration order, all 16-bit
	leas	-8,s
	ldd	#42		; first vararg: int 42
	std	,s
	ldd	#str_ok		; second vararg: const char *"ok"
	std	2,s
	ldd	#0xBEEF		; third vararg: unsigned int 0xBEEF
	std	4,s
	ldd	#0x5A		; fourth vararg: int 'Z'
	std	6,s
	ldx	#str_printf_fmt
	jsr	test_printf
	leas	8,s
; CHECK-NEXT: Hi=42 ok beef Z %

	;; printf("%d\n", -1) — exercise the negative-int path
	;; (this is what triggered the s16→s32 G_SEXT legalizer gap)
	leas	-2,s
	ldd	#-1
	std	,s
	ldx	#str_printf_d
	jsr	test_printf
	leas	2,s
; CHECK-NEXT: -1

	;; printf("%u %x %X\n", 65535, 255, 255)
	leas	-6,s
	ldd	#65535
	std	,s
	ldd	#255
	std	2,s
	ldd	#255
	std	4,s
	ldx	#str_printf_uxX
	jsr	test_printf
	leas	6,s
; CHECK-NEXT: 65535 ff FF

	rts

	.section .rodata,"a",@progbits
str_42:		.asciz	"42"
str_neg3:	.asciz	"-3"
str_atol_0:	.asciz	"0"
str_ff:		.asciz	"FF"
str_0x10:	.asciz	"0x10"
str_777:	.asciz	"777"
str_12345:	.asciz	"12345"
str_ffff:	.asciz	"FFFF"
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
str_abc:	.asciz	"a,b,c"
str_comma:	.asciz	","
str_ok:		.asciz	"ok"
str_printf_fmt:	.asciz	"Hi=%d %s %x %c %%\n"
str_printf_d:	.asciz	"%d\n"
str_printf_uxX:	.asciz	"%u %x %X\n"

	.section .bss,"aw",@nobits
buf:	.space	16
