# REQUIRES: mc6809

# Binary and OS-9 output never give a non-ALLOC section a file offset, so a
# debug section appears to overlap the module body when its place in the
# output is computed from that offset. Relocations in loaded sections must
# still be applied as what they are.

# RUN: rm -rf %t && split-file %s %t && cd %t
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj data.s -o data.o
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj debug.s -o debug.o
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj badref.s -o badref.o

# A pointer word in .data is rewritten to a data-area offset, not taken as
# part of the debug section ahead of it and left as a raw address. The body
# is rts, then .data: ptr -> var at data-area offset $102 (the script puts
# .data at U+$100), then var, then an init-array entry -> _start at body
# offset 0, then the relocation table: kind 2 for ptr at U-offset $100,
# kind 1 for the entry at $104. Left raw, ptr would hold var's body address,
# $0003. The init-array entry is SHT_INIT_ARRAY, a section type whose value
# shares a bit with SHT_NOBITS; it is loaded and must be found as such.
# RUN: ld.lld -T %S/Inputs/mc6809-os9-data-area.lds --os9-name=t --os9-mem=1024 \
# RUN:   data.o -o data.os9
# RUN: xxd -p -c 65536 data.os9 | FileCheck %s
# CHECK: 39010200000000020100010104{{0*}}00

# An OS-9 data-area operand naming a read-only object is a link error, even
# when an earlier object's debug section appears to cover the instruction.
# RUN: not ld.lld -T %S/Inputs/mc6809-os9-data-area.lds --os9-name=t \
# RUN:   --os9-mem=1024 debug.o badref.o -o /dev/null 2>&1 | FileCheck %s --check-prefix=BAD
# BAD: error: badref.o:({{.*}}.text+0x2): OS-9 data-area relocation target is not in .dp.data, .dp.bss, .data or .bss: {{.*}}(.rodata)

#--- data.s
        .section .debug_ranges,"",@progbits
        .space  512

        .text
        .globl _start
_start:
        rts

        .data
ptr:    .word var
var:    .word 0

        .section .data.init,"aw",@init_array
        .word   _start

#--- debug.s
        .section .debug_ranges,"",@progbits
        .space  512

#--- badref.s
        .text
        .globl _start
_start:
        ldx     mc6809_os9_data(ro),u
        rts

        .rodata
ro:     .byte 1
