# Bug #163 Phase 3: byte-exact cross-check between the native lld
# OS-9 writer and tools/os9-link (the Python reference implementation).
#
# Two paths from the same .o → an OS-9 module:
#
#   Path A (legacy): ld.lld --oformat=binary  →  os9-link wrap
#   Path B (native): ld.lld --oformat=os9-program-module
#
# Both paths must produce IDENTICAL bytes for the same input.  This
# sentinel guards against algorithmic drift between the two — if a
# future contributor changes one of the CRC implementations, or
# tweaks the layout in one path but not the other, this test fails
# immediately.  os9-link is the reference; lld must match it.
#
# REQUIRES: mc6809

# A small but non-trivial body: clrb; .byte 0x10,0x3f,0x06; bra .
# Plus a small .rodata to ensure the section layout is non-empty.
# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o

# Path A: binary body, then Python wrap.
# RUN: ld.lld -o %t-A.body %t.o --oformat=binary
# RUN: %S/../../../tools/os9-link %t-A.body \
# RUN:   --name probe --exec 13 --mem 256 -o %t-A.os9

# Path B: native lld writer.
# RUN: ld.lld -o %t-B.os9 %t.o --oformat=os9-program-module \
# RUN:   --os9-name=probe --os9-exec=13 --os9-mem=256

# Byte-exact diff.  If this fails, ONE of the two implementations
# has drifted — debug by hexdumping both and comparing.
# RUN: diff %t-A.os9 %t-B.os9

# Sanity: Phase 1 validator accepts both.
# RUN: %S/../../../tools/os9-module-check %t-A.os9 \
# RUN:   | FileCheck %s --check-prefix=VALID
# RUN: %S/../../../tools/os9-module-check %t-B.os9 \
# RUN:   | FileCheck %s --check-prefix=VALID
# VALID: CRC: valid (full CRC = $800FE3)

# HD6309 variant — same cross-check at type=0x17.
# RUN: ld.lld -o %t-A-hd.body %t.o --oformat=binary
# RUN: %S/../../../tools/os9-link %t-A-hd.body --name probe \
# RUN:   --lang Obj6309 --exec 13 --mem 256 -o %t-A-hd.os9
# RUN: ld.lld -o %t-B-hd.os9 %t.o --oformat=os9-program-module \
# RUN:   --os9-name=probe --os9-type=0x17 --os9-exec=13 --os9-mem=256
# RUN: diff %t-A-hd.os9 %t-B-hd.os9

        .text
        .globl _start
_start:
        clrb
        swi2
        .byte   0x06
1:      bra 1b

        .section .rodata,"a",@progbits
str:
        .ascii  "probe"
        .byte   0
