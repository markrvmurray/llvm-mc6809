# RUN: llvm-mc -triple=mc6809 -show-encoding %s | FileCheck %s
# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o
# RUN: llvm-readobj -r %t.o | FileCheck %s --check-prefix=RELOC

# A relocation modifier is allowed as the constant offset of an indexed
# operand. The offset field is signed and the modifier's value is a
# link-time quantity of the same width, so the operand matches on width, not
# on the (unknown) value: 16-bit modifiers take the 16-bit offset form, 8-bit
# ones the 8-bit form.

        ldx     mc6809_16(table),u
        leay    mc6809_16(table),x
        ldb     mc6809_8(table),y

# CHECK: ldx mc6809_16(table),u   ; encoding: [0xae,0xc9,A,A]
# CHECK-NEXT: fixup A - offset: 2, value: mc6809_16(table), kind: Addr16
# CHECK: leay mc6809_16(table),x  ; encoding: [0x31,0x89,A,A]
# CHECK-NEXT: fixup A - offset: 2, value: mc6809_16(table), kind: Addr16
# CHECK: ldb mc6809_8(table),y    ; encoding: [0xe6,0xa8,A]
# CHECK-NEXT: fixup A - offset: 2, value: mc6809_8(table), kind: Addr8

# RELOC:      Section ({{[0-9]+}}) .rela.text {
# RELOC-NEXT:   0x2 R_MC6809_ADDR_16 table 0x0
# RELOC-NEXT:   0x6 R_MC6809_ADDR_16 table 0x0
# RELOC-NEXT:   0xA R_MC6809_ADDR_8 table 0x0
# RELOC-NEXT: }

        .data
        .globl table
table:
        .zero 4
