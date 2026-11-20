; RUN: echo '.include "runtime.inc"' > %t-rt.s
; RUN: llvm-mc -triple=mc6809 -I %S/Inputs --filetype=obj -o %t-rt.o %t-rt.s
; RUN: %clang6809 -O0 -c -o %t-O0.o %S/Inputs/harness-umulo.c
; RUN: ld.lld -T %S/Inputs/link.ld %t-rt.o %t-O0.o %mc6809_builtins -o %t-O0.elf
; RUN: llvm-objcopy -O ihex %t-O0.elf %t-O0.hex
; RUN: %usim09batch --timeout=100000000 %t-O0.hex | FileCheck %s
; RUN: %clang6809 -Os -c -o %t-Os.o %S/Inputs/harness-umulo.c
; RUN: ld.lld -T %S/Inputs/link.ld %t-rt.o %t-Os.o %mc6809_builtins -o %t-Os.elf
; RUN: llvm-objcopy -O ihex %t-Os.elf %t-Os.hex
; RUN: %usim09batch --timeout=100000000 %t-Os.hex | FileCheck %s
; RUN: %clang6809 -O2 -c -o %t-O2.o %S/Inputs/harness-umulo.c
; RUN: ld.lld -T %S/Inputs/link.ld %t-rt.o %t-O2.o %mc6809_builtins -o %t-O2.elf
; RUN: llvm-objcopy -O ihex %t-O2.elf %t-O2.hex
; RUN: %usim09batch --timeout=100000000 %t-O2.hex | FileCheck %s
;
; The 6309 build of the same, run as a 6309.
; RUN: echo '.include "runtime6309.inc"' > %t-rt6309.s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -I %S/Inputs --filetype=obj -o %t-rt6309.o %t-rt6309.s
; RUN: %clang6809 -mcpu=hd6309 -Os -c -o %t-6309.o %S/Inputs/harness-umulo.c
; RUN: ld.lld -T %S/Inputs/link.ld %t-rt6309.o %t-6309.o %mc6809_builtins -o %t-6309.elf
; RUN: %hd6309sim %t-6309.elf | FileCheck %s
;
; Unsigned multiply with overflow (__builtin_mul_overflow), which calloc,
; fread and fwrite use to check a size: one MUL at 8 bits, __umulohi4 at 16,
; and at 32 bits __mulsi3 plus a 16-bit check of the cross product. Every
; pair of bytes, and edge-biased pseudo-random pairs at 16 and 32 bits,
; checked against a multiply twice as wide (Inputs/harness-umulo.c).

; CHECK: 1D4C
; CHECK-NEXT: 0000
