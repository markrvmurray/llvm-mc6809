// An object says in its ELF header which processor its code needs.  A 6309
// runs 6809 code, so it is the other direction that matters: without this an
// object built for the 6309 looks exactly like a 6809 one, and whatever runs
// it -- a simulator, a loader, a disassembler choosing a decode table -- has
// to be told separately, or guess and be wrong.

// RUN: llvm-mc -filetype=obj -triple=mc6809 %s -o %t.6809.o
// RUN: llvm-readobj -h %t.6809.o | FileCheck %s --check-prefix=MC6809
// MC6809: Flags [ (0x1)
// MC6809-NEXT: EF_MC6809_ARCH_6809 (0x1)

// RUN: llvm-mc -filetype=obj -triple=mc6809 -mcpu=hd6309 %s -o %t.6309.o
// RUN: llvm-readobj -h %t.6309.o | FileCheck %s --check-prefix=HD6309
// HD6309: Flags [ (0x2)
// HD6309-NEXT: EF_MC6809_ARCH_6309 (0x2)

        lda #1
        rts
