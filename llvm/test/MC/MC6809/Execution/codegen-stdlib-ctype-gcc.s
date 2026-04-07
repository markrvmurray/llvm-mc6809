; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-stdlib-ctype.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-stdlib-ctype.c
; RUN: %S/Inputs/sdcc2gas.sh < %t-harness-raw.s > %t-harness.s
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-harness.s %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=1000000 %t.hex | FileCheck %s
; REQUIRES: usim, gcc6809
;
; Cross-compiler execution test: C harness compiled by gcc6809, functions
; under test compiled by LLVM MC6809. Validates calling convention interop.
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
