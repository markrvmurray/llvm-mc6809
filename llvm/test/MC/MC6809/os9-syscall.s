# RUN: llvm-mc -triple=mc6809-unknown-os9 -show-encoding %s | FileCheck %s
# RUN: llvm-mc -triple=mc6809 -mattr=+os9 -show-encoding %s | FileCheck %s
# RUN: not llvm-mc -triple=mc6809 %s 2>&1 | FileCheck %s --check-prefix=BARE
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj %s -o %t.o
# RUN: llvm-objdump -d --triple=mc6809-unknown-os9 %t.o | FileCheck %s --check-prefix=DIS
# RUN: llvm-objdump -d --triple=mc6809 %t.o | FileCheck %s --check-prefix=DIS-BARE

# `os9 <code>` is SWI2 followed by the in-line function-code byte, the way
# lwasm and the Microware assembler spell an OS-9 system call. It exists only
# in the OS-9 environment (the os9 triple, or +os9), where SWI2 is the kernel
# entry. The code is a bare expression -- symbolic or numeric -- or, for the
# benefit of C inline asm and habit, an immediate.

F$Exit = 6
I$Write = 0x8A

        os9 F$Exit
        os9 I$Write
        os9 0x89
        os9 #$8A
        swi2
        .byte 6

# CHECK: os9 6      ; encoding: [0x10,0x3f,0x06]
# CHECK: os9 138    ; encoding: [0x10,0x3f,0x8a]
# CHECK: os9 137    ; encoding: [0x10,0x3f,0x89]
# CHECK: os9 138    ; encoding: [0x10,0x3f,0x8a]
# CHECK: swi2       ; encoding: [0x10,0x3f]

# BARE: error: instruction requires a CPU feature not currently enabled

# Disassembling for OS-9 renders every SWI2 with its function code as one
# construct, so the byte after SWI2 is not mistaken for an opcode.
# DIS:      os9 $6
# DIS-NEXT: os9 $8a
# DIS-NEXT: os9 $89
# DIS-NEXT: os9 $8a
# DIS-NEXT: os9 $6

# Without the environment the stream is what it is: SWI2, then bytes.
# DIS-BARE: swi2
# DIS-BARE-NOT: os9
