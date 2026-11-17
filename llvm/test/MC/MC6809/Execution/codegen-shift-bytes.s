; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift-bytes.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-shift-bytes.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift-bytes.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-shift-bytes.c
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
; Constant 16-bit shifts by eight or more and signed divisions by a power of
; two, against values worked out independently, on both processors. Each line
; is one input: x<<8, x<<11, x>>8, x>>13 (unsigned), x>>8, x>>12, x>>15,
; x/4, x/256 (signed), and a byte zero-extended and shifted left by 9.
;
; CHECK: 0000000000000000000000000000000000000000
; CHECK-NEXT: 0100080000000000000000000000000000000200
; CHECK-NEXT: 0300180000000000000000000000000000000600
; CHECK-NEXT: FF00F80000FF0007FFFFFFFFFFFF00000000FE00
; CHECK-NEXT: FD00E80000FF0007FFFFFFFFFFFF00000000FA00
; CHECK-NEXT: FF00F80000000000000000000000003F0000FE00
; CHECK-NEXT: 0000000000FF0007FFFFFFFFFFFFFFC0FFFF0000
; CHECK-NEXT: 3400A00000120000001200010000048D00126800
; CHECK-NEXT: CC00600000ED0007FFEDFFFEFFFFFB73FFEE9800
; CHECK-NEXT: FF00F800007F0003007F000700001FFF007FFE00
; CHECK-NEXT: 0000000000800004FF80FFF8FFFFE000FF800000
; CHECK-NEXT: FC00E00000FF0007FFFFFFFFFFFFFFFF0000F800
; CHECK-NEXT: FB00D80000FF0007FFFFFFFFFFFFFFFF0000F600
