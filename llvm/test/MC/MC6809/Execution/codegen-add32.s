; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-add32.c
; RUN: %S/Inputs/sdcc2gas.sh < %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-add32.c
; RUN: %S/Inputs/sdcc2gas.sh < %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-add32.c
; RUN: %S/Inputs/sdcc2gas.sh < %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-add32.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-add32.c
; RUN: %S/Inputs/sdcc2gas.sh < %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=500000 %t.hex | FileCheck %s
; REQUIRES: usim, gcc6809
;
; Codegen execution test: 32-bit addition. Tests both the general
; two-arg form (add_i32) and the add-by-constant form (add_i32_const).
; The harness (Inputs/harness-add32.c) is compiled by gcc6809 and the
; functions under test (Inputs/codegen-add32.ll) by LLVM-MC6809.
; The two i32-returning functions are called via void-returning
; out-pointer wrappers (add_i32_w, add_i32_const_w) defined in the
; .ll, because gcc6809 and LLVM-MC6809 use incompatible long-return
; conventions.

; CHECK: 00000003
; CHECK-NEXT: 00010000
; CHECK-NEXT: 80000000
; CHECK-NEXT: 00000000
; CHECK-NEXT: ACF13568
; CHECK-NEXT: 00007531
; CHECK-NEXT: FFFF7530
; CHECK-NEXT: ACF13568
