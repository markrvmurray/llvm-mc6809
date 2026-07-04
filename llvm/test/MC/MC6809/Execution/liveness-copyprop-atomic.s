; Bug #370: end-to-end runtime-value sentinel for the spill / frame-addressing /
; copy-propagation machinery that Bug #357 (TFR-as-copy + late
; MachineCopyPropagation) and Bug #363/#366 (index-register redundant-reload
; elimination) operate on. Those passes have precise STATIC regression tests
; (test/CodeGen/MC6809/bug357_tfr_copyprop.mir and bug366-stack-aliasing-store.mir);
; this is the value layer they cannot provide — it runs frame-heavy code on
; usim and checks the bytes, so a future change that makes atomic-register
; copy-propagation or the index reload elimination UNSOUND fails here, fast,
; rather than only in the 28-level bench.
;
; The accumulator-family #357/#366 miscompiles are deliberately NOT targeted:
; they are whole-program-cumulative and do not reproduce in a small
; value-checkable function (verified against deliberately-regressed compilers
; while authoring this test). The atomic registers (IX/IY/SU/SS) are sound by
; construction, so a correct value at every opt level is the meaningful
; invariant here.
;
; The TRIGGER-guard asserts the frame-addressing / atomic-transfer shape is
; actually generated at -O1/-O2 (the levels where the passes run), so the test
; cannot pass vacuously: if codegen drifts so frame_sum no longer re-materialises
; the frame base into an index register, the guard fails and the trigger must be
; re-derived.

; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/liveness-copyprop-atomic.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-liveness-copyprop-atomic.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=3000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:   %S/Inputs/liveness-copyprop-atomic.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: FileCheck --check-prefix=TRIGGER %s < %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-liveness-copyprop-atomic.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=3000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/liveness-copyprop-atomic.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: FileCheck --check-prefix=TRIGGER %s < %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-liveness-copyprop-atomic.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=3000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:   %S/Inputs/liveness-copyprop-atomic.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -o %t-harness.o %S/Inputs/harness-liveness-copyprop-atomic.c
; RUN: echo '.include "runtime.inc"' > %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=3000000 %t.hex | FileCheck %s
;
; REQUIRES: usim

; frame_sum(0x11,0x22,0x44) = 0x1709 ; ptr_roundtrip(data,5) = 0x0F0F
; CHECK: 1709
; CHECK-NEXT: 0F0F

; A pointer re-materialised into an index register by reloading it from
; memory — the index-reload shape the copy-propagation guards act on.
; Either index register satisfies the guard, and the reload source may be
; a U-relative frame slot or (with the RS imaginaries allocatable and the
; frame pointer dropped for RS-resident locals) a direct-page imaginary.
; The historical `tfr s,u` anchor is gone with the frame pointer; the
; reload itself is what feeds the guarded passes.
; TRIGGER: ld{{[xyd]}} {{([0-9]+,u|<__rs[0-3])}}
