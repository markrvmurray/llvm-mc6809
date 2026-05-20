; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/cc_funcs_torture.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-cc-torture.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/cc_funcs_torture.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-cc-torture.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/cc_funcs_torture.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-cc-torture.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/cc_funcs_torture.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-cc-torture.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=2000000 %t.hex | FileCheck %s
; REQUIRES: usim
;
; Calling-convention torture set: drives the functions in
; cc_funcs_torture.ll through harness-cc-torture.c with known
; sentinel values for i32 and i64, then verifies the printed hex
; results.  Each function isolates one ABI shape.  See the IR file
; for the shape catalogue.
;
; Note about CHECK lines: the actual expected hex strings live as
; CHECK comments INSIDE harness-cc-torture.c (one per call) so the
; sentinel value and its expected output appear side-by-side at the
; call site.  FileCheck reads them from STDIN (the usim output) and
; uses %s (this file) only to find further CHECK directives — which
; we leave to the harness.  The grouped check block below is just a
; safety net that catches a total ordering failure.

; CHECK:      CAFEF00D
; CHECK-NEXT: CAFEF00D
; CHECK-NEXT: CAFEF00D
; CHECK-NEXT: CAFEF00D
; CHECK-NEXT: F00D
; CHECK-NEXT: CAFE
; CHECK-NEXT: 0D
; CHECK-NEXT: CA
; CHECK-NEXT: FFEEDDCCBBAA9988
; CHECK-NEXT: BBAA9988
; CHECK-NEXT: FFEEDDCC
; CHECK-NEXT: 88
; CHECK-NEXT: FF
; CHECK-NEXT: FFEEDDCCBBAA9988
; CHECK-NEXT: FFEEDDCCBBAA9988
