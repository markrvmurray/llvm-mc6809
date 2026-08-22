# REQUIRES: mc6809

# OS-9 data-area relocations. The linker script says, for each writable
# region, where its image is in the module body and where the region sits in
# the process data area (a U-relative offset). R_MC6809_OS9_OFFSET_16 and
# R_MC6809_OS9_OFFSET_8 resolve to that offset; the 8-bit form is the
# direct-page operand for an object in the first page of the data area, and is
# an error for anything else. A pointer word in .data or .dp.data is rewritten
# to an offset too (into the data area or into the body) and listed in the
# .os9_reloc table for the CRT: kind 2 = add U, kind 1 = add the module base.

# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj %s -o %t.o
# RUN: ld.lld -T %S/Inputs/mc6809-os9-data-area.lds --os9-name=t --os9-mem=1024 \
# RUN:   %t.o -o %t.os9
# RUN: xxd -p -c 65536 %t.os9 | FileCheck %s

# The two operands, then the .dp.data image, then the .data image, then the
# relocation table; the header, name and CRC are not pinned.
#   ldd <off8(dpvar)     dc 20      dpvar is at data-area offset $20
#   ldx off16(bigvar),u  ae c9 0100 bigvar is at data-area offset $0100
#   .dp.data: dp_ptr -> bigvar   00 01 00 (offset $0100, kind 2 at U-offset $21)
#   .data:    p_dp -> dpvar      00 20    (kind 2 at U-offset $100+2 = $0102)
#             p_ro -> ro         xxxx    (body offset, kind 1 at $0104)
#   .os9_reloc: 02 0021  02 0102  01 0104  00
# CHECK: dc20aec90100{{[0-9a-f]*}}000100{{[0-9a-f]*}}0020{{[0-9a-f][0-9a-f][0-9a-f][0-9a-f]}}02002102010201010400

# An 8-bit operand for an object outside the first page is a link error.
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj \
# RUN:   --defsym FAR=1 %s -o %t-far.o
# RUN: not ld.lld -T %S/Inputs/mc6809-os9-data-area.lds --os9-name=t \
# RUN:   --os9-mem=1024 %t-far.o -o /dev/null 2>&1 | FileCheck %s --check-prefix=FAR
# FAR: error: {{.*}}OS-9 direct-page operand: object is not in the first page of the data area (offset 258)

        .text
        .globl _start
_start:
        ldd     <mc6809_os9_data8(dpvar)
        ldx     mc6809_os9_data(bigvar),u
.ifdef FAR
        ldb     <mc6809_os9_data8(bigvar+2)
.endif
        rts

        .rodata
ro:     .byte 1

        .section .dp.data,"aw",@progbits
dpvar:  .byte 0
dp_ptr: .word bigvar

        .data
bigvar: .word 0
p_dp:   .word dpvar
p_ro:   .word ro
