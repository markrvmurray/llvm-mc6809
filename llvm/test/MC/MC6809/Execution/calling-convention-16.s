; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/cc_funcs16.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-cc16.c
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
; 16-bit calling convention validation.
;
; gcc6809 (the reference compiler) compiles harness-cc16.c into a
; driver that calls the LLVM-compiled cc_funcs16.ll directly. Both
; halves target ABI version 1: arg dispatch is by *type* with two
; independent register slots — the first i8 arg fills B, the first
; i16 arg fills X, anything past those goes on the stack (i8 in
; 1-byte slots, i16 in 2-byte slots). Return i16 in X. The functions
; here take only i16 args, so for f(i16, i16) the first arg fills X
; and the second is pushed via `stx ,--s` (sitting at S+2 after jsr).
; If LLVM-MC6809 diverges from gcc6809 on i16 layout the link works
; but the runtime output mismatches FileCheck — that's the failure
; mode this test catches.
;
; CHECK: BEEF
; CHECK-NEXT: 5555
; CHECK-NEXT: 7FFF
