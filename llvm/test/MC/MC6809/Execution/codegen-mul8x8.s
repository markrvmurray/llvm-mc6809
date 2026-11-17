; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul8x8.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-mul8x8.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-mul8x8.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-mul8x8.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --hd6309 --timeout=5000000 %t.hex | FileCheck %s
;
; A grid of unsigned byte products, then each byte times 200, then products
; through loads, against results worked out independently. Both processors.
;
; CHECK: 000000000000000000000000000000000000000000000000
; CHECK-NEXT: 0000000100020003000F0010007F0080008100C800FE00FF
; CHECK-NEXT: 0000000200040006001E002000FE01000102019001FC01FE
; CHECK-NEXT: 0000000300060009002D0030017D01800183025802FA02FD
; CHECK-NEXT: 0000000F001E002D00E100F007710780078F0BB80EE20EF1
; CHECK-NEXT: 000000100020003000F0010007F0080008100C800FE00FF0
; CHECK-NEXT: 0000007F00FE017D077107F03F013F803FFF63387E027E81
; CHECK-NEXT: 0000008001000180078008003F804000408064007F007F80
; CHECK-NEXT: 0000008101020183078F08103FFF4080410164C87FFE807F
; CHECK-NEXT: 000000C8019002580BB80C806338640064C89C40C670C738
; CHECK-NEXT: 000000FE01FC02FA0EE20FE07E027F007FFEC670FC04FD02
; CHECK-NEXT: 000000FF01FE02FD0EF10FF07E817F80807FC738FD02FE01
; CHECK-NEXT: 000000C8019002580BB80C806338640064C89C40C670C738
; CHECK-NEXT: 000000FE01900183078007F007F007800183019000FE0000
