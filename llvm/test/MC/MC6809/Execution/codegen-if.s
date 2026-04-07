; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-if.c
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
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-if.c
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
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-if.c
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
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-if.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-if.c
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
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim, gcc6809
;
; Codegen execution test: conditionals (icmp + select) for i8/i16/i32.
; The harness (Inputs/harness-if.c) is compiled by gcc6809 and the
; functions under test by LLVM-MC6809; linking the two validates
; calling-convention interop end-to-end at every opt level.
;
; The two i32-returning functions (test_max_s32, test_min_u32) are
; called via void-returning out-pointer wrappers (test_max_s32_w,
; test_min_u32_w) defined in codegen-if.ll, because gcc6809 and
; LLVM-MC6809 have incompatible long-return conventions.

; CHECK: 05
; CHECK-NEXT: 05
; CHECK-NEXT: 01
; CHECK-NEXT: 07
; CHECK-NEXT: 03E8
; CHECK-NEXT: 0001
; CHECK-NEXT: FF
; CHECK-NEXT: FFFF
; CHECK-NEXT: 03
; CHECK-NEXT: FF
; CHECK-NEXT: 01
; CHECK-NEXT: 00
; CHECK-NEXT: 01
; CHECK-NEXT: 00
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 00000003
; CHECK-NEXT: 00000003
; CHECK-NEXT: 00000001
; CHECK-NEXT: 12345679
; CHECK-NEXT: 00000003
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
