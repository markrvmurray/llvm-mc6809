; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mulh.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-mulh.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mulh.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-mulh.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 \
; RUN:   %S/Inputs/codegen-mulh.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-mulh.c
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
; High half of a widening multiply, unsigned then signed, at 8, 16 and 32 bits.
;
; CHECK: 00
; CHECK-NEXT: 00
; CHECK-NEXT: 03
; CHECK-NEXT: 3F
; CHECK-NEXT: 40
; CHECK-NEXT: 00
; CHECK-NEXT: FE
; CHECK-NEXT: 7E
; CHECK-NEXT: 00
; CHECK-NEXT: 00
; CHECK-NEXT: 03
; CHECK-NEXT: C0
; CHECK-NEXT: 40
; CHECK-NEXT: FF
; CHECK-NEXT: 00
; CHECK-NEXT: FF
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0626
; CHECK-NEXT: 3FFF
; CHECK-NEXT: 4000
; CHECK-NEXT: 0000
; CHECK-NEXT: FFFE
; CHECK-NEXT: 0001
; CHECK-NEXT: 00FE
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0626
; CHECK-NEXT: C000
; CHECK-NEXT: 4000
; CHECK-NEXT: FFFF
; CHECK-NEXT: 0000
; CHECK-NEXT: 0001
; CHECK-NEXT: FFFF
; CHECK-NEXT: 00000000
; CHECK-NEXT: 00000000
; CHECK-NEXT: 0B00EA4E
; CHECK-NEXT: 3FFFFFFF
; CHECK-NEXT: 40000000
; CHECK-NEXT: 00000000
; CHECK-NEXT: FFFFFFFE
; CHECK-NEXT: 00000001
; CHECK-NEXT: 0000FFFE
; CHECK-NEXT: 00000000
; CHECK-NEXT: 00000000
; CHECK-NEXT: F8CC93D6
; CHECK-NEXT: C0000000
; CHECK-NEXT: 40000000
; CHECK-NEXT: FFFFFFFF
; CHECK-NEXT: 00000000
; CHECK-NEXT: 00000001
; CHECK-NEXT: FFFFFFFF
