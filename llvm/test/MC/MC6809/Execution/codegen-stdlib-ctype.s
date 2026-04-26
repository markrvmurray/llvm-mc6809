; This test runs `Inputs/codegen-stdlib-ctype.ll` end-to-end on USim
; with the harness in `Inputs/harness-stdlib-ctype.c`. Among other
; things, it exercises `test_atoi_neg` — the historical repro for
; bugs #49 / #159 / #166 (silent PHI-D-clobber across an i8 byte
; load + i8 compare). If `test_atoi_neg("-3")` returns anything
; other than -3, that PHI-D-clobber concern has reappeared. The
; codegen-shape-level sentinel for the same concern is
; `test/CodeGen/MC6809/cmp_imm_byte_load_phi.ll` — keep both in
; sync if you ever need to refactor either.
;
; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-stdlib-ctype.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-stdlib-ctype.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-stdlib-ctype.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-stdlib-ctype.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Cross-compiler execution test: C harness (Inputs/harness-stdlib-ctype.c)
; compiled by gcc6809, functions under test (Inputs/codegen-stdlib-ctype.ll)
; compiled by LLVM-MC6809. Tested at -O0 through -O3. The previous
; hand-written asm test driver (codegen-stdlib-ctype.s in this same
; location) is now retired — its 24 test cases are all covered by the
; C harness with identical CHECK lines.
;
; abs
; CHECK: 0005
; CHECK-NEXT: 0003
; CHECK-NEXT: 0000
;
; atoi (positive, bug #52 fix)
; CHECK-NEXT: 002A
; CHECK-NEXT: 0000
; CHECK-NEXT: 03E7
;
; atoi_neg (positive and negative, bug #49 fix)
; CHECK-NEXT: 002A
; CHECK-NEXT: FFFD
; CHECK-NEXT: 0000
; CHECK-NEXT: FFFF
; CHECK-NEXT: FC19
;
; div: 17 / 5 = 3 rem 2
; CHECK-NEXT: 00030002
;
; 16-bit bitwise
; CHECK-NEXT: FFFF
; CHECK-NEXT: 0F00
;
; ctype
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: 0000
