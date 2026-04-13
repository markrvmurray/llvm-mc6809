; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-picolibc-all.c
; RUN: perl -ne 'next if /^\s*\.module/; \
; RUN:   s/\.area\s+\.text/.section .rom,"ax",\@progbits/; \
; RUN:   s/\.area\s+\.bss/.section .bss,"aw",\@nobits/; \
; RUN:   s/\.area\s+\.rodata/.section .rodata,"a",\@progbits/; \
; RUN:   s/(?<![a-zA-Z0-9])_([a-zA-Z])/$1/g; \
; RUN:   s/\.blkb\s+(\d+)/.space $1/; s/\bzmb\s+(\d+)/.space $1/; \
; RUN:   print;' %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-picolibc-all.c
; RUN: perl -ne 'next if /^\s*\.module/; \
; RUN:   s/\.area\s+\.text/.section .rom,"ax",\@progbits/; \
; RUN:   s/\.area\s+\.bss/.section .bss,"aw",\@nobits/; \
; RUN:   s/\.area\s+\.rodata/.section .rodata,"a",\@progbits/; \
; RUN:   s/(?<![a-zA-Z0-9])_([a-zA-Z])/$1/g; \
; RUN:   s/\.blkb\s+(\d+)/.space $1/; s/\bzmb\s+(\d+)/.space $1/; \
; RUN:   print;' %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-picolibc-all.c
; RUN: perl -ne 'next if /^\s*\.module/; \
; RUN:   s/\.area\s+\.text/.section .rom,"ax",\@progbits/; \
; RUN:   s/\.area\s+\.bss/.section .bss,"aw",\@nobits/; \
; RUN:   s/\.area\s+\.rodata/.section .rodata,"a",\@progbits/; \
; RUN:   s/(?<![a-zA-Z0-9])_([a-zA-Z])/$1/g; \
; RUN:   s/\.blkb\s+(\d+)/.space $1/; s/\bzmb\s+(\d+)/.space $1/; \
; RUN:   print;' %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-picolibc.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-picolibc-all.c
; RUN: perl -ne 'next if /^\s*\.module/; \
; RUN:   s/\.area\s+\.text/.section .rom,"ax",\@progbits/; \
; RUN:   s/\.area\s+\.bss/.section .bss,"aw",\@nobits/; \
; RUN:   s/\.area\s+\.rodata/.section .rodata,"a",\@progbits/; \
; RUN:   s/(?<![a-zA-Z0-9])_([a-zA-Z])/$1/g; \
; RUN:   s/\.blkb\s+(\d+)/.space $1/; s/\bzmb\s+(\d+)/.space $1/; \
; RUN:   print;' %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
; REQUIRES: usim, gcc6809
;
; Picolibc execution test — full coverage of the 36 test_* functions
; defined in Inputs/codegen-picolibc.ll. The C harness driver
; (Inputs/harness-picolibc-all.c) is compiled by gcc6809 and the
; functions under test by LLVM-MC6809; linking the two validates
; calling-convention interop end-to-end at every opt level.
;
; Functions covered:
;   string.h:  memcpy, memset, memmove, memcmp, memchr, strcpy, strncpy,
;              strcat, strncat, strlen, strchr, strrchr, strstr, strspn,
;              strcspn, strpbrk, strncmp, strtok
;   ctype.h:   isspace, isprint, isxdigit, tolower, toupper, ispunct,
;              iscntrl, isgraph
;   stdlib.h:  labs, atol, strtol, strtoul, rand, srand
;   stdio.h:   putchar, printf
;
; Long-returning functions (labs/atol/strtol/strtoul) and printf are
; called via small bridge wrappers in codegen-picolibc.ll because
; gcc6809's calling convention for those cases is incompatible with
; LLVM-MC6809's. See the wrappers near the bottom of the .ll file
; (test_*_w, test_printf_*) and the comment in harness-picolibc-all.c
; for the rationale.

; CHECK: 48656C6C6F
; CHECK-NEXT: AAAAAAAA
; CHECK-NEXT: 48656C6C6F
; CHECK-NEXT: 4848656C6C6F
; CHECK-NEXT: 486900
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0041
; CHECK-NEXT: 005A
; CHECK-NEXT: 0000
; CHECK-NEXT: FFFC
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0061
; CHECK-NEXT: 007A
; CHECK-NEXT: 0002
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: FFFC
; CHECK-NEXT: 0005
; CHECK-NEXT: 0003
; CHECK-NEXT: 0000
; CHECK-NEXT: 0002
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0002
; CHECK-NEXT: 0000
; CHECK-NEXT: 0004
; CHECK-NEXT: 0002
; CHECK-NEXT: 0002
; CHECK-NEXT: 48690000
; CHECK-NEXT: 4865792100
; CHECK-NEXT: 4869686F00
; CHECK-NEXT: 0000
; CHECK-NEXT: 0002
; CHECK-NEXT: 0004
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 00000005
; CHECK-NEXT: 00000000
; CHECK-NEXT: 00000003
; CHECK-NEXT: 0C5F
; CHECK-NEXT: 1FE6
; CHECK-NEXT: 5873
; CHECK-NEXT: 005C
; CHECK-NEXT: 0000002A
; CHECK-NEXT: FFFFFFFD
; CHECK-NEXT: 00000000
; CHECK-NEXT: 0000002A
; CHECK-NEXT: FFFFFFFD
; CHECK-NEXT: 000000FF
; CHECK-NEXT: 00000010
; CHECK-NEXT: 000001FF
; CHECK-NEXT: 00003039
; CHECK-NEXT: 0000FFFF
; CHECK-NEXT: !
; CHECK-NEXT: Hi=42 ok beef Z %
; CHECK-NEXT: -1
; CHECK-NEXT: 65535 ff FF
