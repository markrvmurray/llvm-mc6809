; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-shift.c
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
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-shift.c
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
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-shift.c
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
; RUN:   %S/Inputs/codegen-shift.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %gcc6809 -S -Os -o %t-harness-raw.s %S/Inputs/harness-shift.c
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
; Codegen execution test: i8 and i16 shifts (shl/lshr/ashr), both
; constant-amount (handled inline by the isel) and variable-amount
; (lowered to libcalls in shiftqi3.inc / shifthi3.inc). The harness
; (Inputs/harness-shift.c) is compiled by gcc6809 and the functions
; under test by LLVM-MC6809; linking the two validates the i8 and
; i16 ABI plus the libcall emission and runtime library together
; at every opt level.

; CHECK: 84
; CHECK-NEXT: F0
; CHECK-NEXT: 42
; CHECK-NEXT: 0F
; CHECK-NEXT: C0
; CHECK-NEXT: 20
; CHECK-NEXT: F8
; CHECK-NEXT: 07
; CHECK-NEXT: 2468
; CHECK-NEXT: 0002
; CHECK-NEXT: 1234
; CHECK-NEXT: 4000
; CHECK-NEXT: C000
; CHECK-NEXT: 0ABC
; CHECK-NEXT: F800
; CHECK-NEXT: 80
; CHECK-NEXT: 01
; CHECK-NEXT: F8
; CHECK-NEXT: 2340
; CHECK-NEXT: 00AB
; CHECK-NEXT: FF80
