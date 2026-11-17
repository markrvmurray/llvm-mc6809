; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-incdec-hoist.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-incdec-hoist.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-incdec-hoist.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-incdec-hoist.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --hd6309 --timeout=5000000 %t.hex | FileCheck %s
;
; REQUIRES: usim
;
; An update of *p that follows a call writing the same byte uses the value
; the call left, and a read ahead of the update sees the value before it.
;
; CHECK: 65
; CHECK-NEXT: 63
; CHECK-NEXT: 0506
