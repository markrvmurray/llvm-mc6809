; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-cmp32-all.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-cmp32-all.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-cmp32-all.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-cmp32-all.c
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
; Each line is one left operand against every right operand: two bytes per
; pair, the eight signed and unsigned predicates of cmp_all and the two of
; cmp_ug, against results worked out independently. Both processors.
;
; CHECK: A902CE00F200CE00F200CE00CE00F200CE00
; CHECK-NEXT: 3203A902F200CE00F200CE00CE00F200CE00
; CHECK-NEXT: 0E030E03A9020E0332030E030E0332030E03
; CHECK-NEXT: 32033203F200A902F20032033203F2003203
; CHECK-NEXT: 0E030E03CE000E03A9020E030E03CE000E03
; CHECK-NEXT: 32033203F200CE00F200A9023203F200CE00
; CHECK-NEXT: 32033203F200CE00F200CE00A902F200CE00
; CHECK-NEXT: 0E030E03CE000E0332030E030E03A9020E03
; CHECK-NEXT: 32033203F200CE00F20032033203F200A902
