; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-interrupt.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-interrupt.c
; RUN: echo '__runtime_irq_vectors = 1' > %t-all.s
; RUN: echo '.include "runtime.inc"' >> %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: echo '.include "irq-probe.inc"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-interrupt.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-interrupt.c
; RUN: echo '__runtime_irq_vectors = 1' > %t-all.s
; RUN: echo '.include "runtime.inc"' >> %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: echo '.include "irq-probe.inc"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --timeout=5000000 %t.hex | FileCheck %s
;
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mcpu=hd6309 -mtriple=mc6809 \
; RUN:   %S/Inputs/codegen-interrupt.ll -o %t-raw.s 2>/dev/null
; RUN: grep -v '\.directpage' %t-raw.s > %t-funcs.s
; RUN: %clang6809 -Oz -c -I %S/Inputs -o %t-harness.o %S/Inputs/harness-interrupt.c
; RUN: echo '__runtime_irq_vectors = 1' > %t-all.s
; RUN: echo '__runtime_native_mode = 1' >> %t-all.s
; RUN: echo '.include "runtime.inc"' >> %t-all.s
; RUN: echo '.include "mc6809rt.s"' >> %t-all.s
; RUN: echo '.include "irq-probe.inc"' >> %t-all.s
; RUN: cat %t-funcs.s >> %t-all.s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj -o %t.o %t-all.s
; RUN: ld.lld -T %S/Inputs/link.ld %t.o %t-harness.o -o %t.elf
; RUN: llvm-objcopy -O ihex %t.elf %t.hex
; RUN: %usim09batch --hd6309 --timeout=5000000 %t.hex | FileCheck %s
;
; REQUIRES: usim
;
; Handlers for an IRQ from the ACIA, a software interrupt and a FIRQ. The
; first four lines are the probes in irq-probe.inc, which fill the registers
; a handler must preserve, take the interrupt and check them: 0 means all
; came back. Two of the handlers call a routine that overwrites every
; register a call may. The simulator has no FIRQ source, so that probe enters
; its handler as the processor would, twice, with a leaf and with a handler
; that calls. Then a 32-bit sum interrupted halfway, the number of interrupts
; taken, and what the handlers computed. The 6309 runs in native mode, the
; only mode a handler supports there.
;
; CHECK: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 0000
; CHECK-NEXT: 11233345
; CHECK-NEXT: 11233345
; CHECK-NEXT: 020202
; CHECK-NEXT: 02060A0E
; CHECK-NEXT: 00020608
; CHECK-NEXT: 0002050E
