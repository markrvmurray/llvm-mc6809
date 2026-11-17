; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-bytes.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-shift32-bytes.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift32-bytes.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-shift32-bytes.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --hd6309 --timeout=5000000 %t.hex | FileCheck %s
;
; One line per operand: shl 8, 16, 24; lshr 8, 16, 24; ashr 8, 16, 24;
; the low byte and the low two bytes sign-extended,
; against results worked out independently, on both processors (the
; 6309 still calls the library for the shifts).
;
; CHECK: 34567800 56780000 78000000 00123456 00001234 00000012 00123456 00001234 00000012 00000078 00005678 
; CHECK-NEXT: 65432100 43210000 21000000 00876543 00008765 00000087 FF876543 FFFF8765 FFFFFF87 00000021 00004321 
; CHECK-NEXT: FFFFFF00 FFFF0000 FF000000 00FFFFFF 0000FFFF 000000FF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF 
; CHECK-NEXT: 00000000 00000000 00000000 00800000 00008000 00000080 FF800000 FFFF8000 FFFFFF80 00000000 00000000 
; CHECK-NEXT: 00807F00 807F0000 7F000000 00000080 00000000 00000000 00000080 00000000 00000000 0000007F FFFF807F 
