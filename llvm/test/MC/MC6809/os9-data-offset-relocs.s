# RUN: llvm-mc -triple=mc6809-unknown-os9 -show-encoding %s | FileCheck %s
# RUN: llvm-mc -triple=mc6809-unknown-os9 -filetype=obj %s -o %t.o
# RUN: llvm-readobj -r %t.o | FileCheck %s --check-prefix=RELOC

# An OS-9 program's writable data lives in a per-process data area addressed
# from U; mc6809_os9_data(sym) is the symbol's 16-bit U-relative offset and
# mc6809_os9_data8(sym) its 8-bit offset for the first page of the data area,
# which the kernel sets DP to. Both are known only to the linker.

        ldx     mc6809_os9_data(counter),u
        ldd     <mc6809_os9_data8(counter)
        stb     <mc6809_os9_data8(flags)
        lda     #mc6809_os9_data8(flags)

# CHECK: ldx mc6809_os9_data(counter),u   ; encoding: [0xae,0xc9,A,A]
# CHECK-NEXT: fixup A - offset: 2, value: mc6809_os9_data(counter), kind: Addr16
# CHECK: ldd <mc6809_os9_data8(counter)   ; encoding: [0xdc,A]
# CHECK-NEXT: fixup A - offset: 1, value: mc6809_os9_data8(counter), kind: Addr8
# CHECK: stb <mc6809_os9_data8(flags)     ; encoding: [0xd7,A]
# CHECK-NEXT: fixup A - offset: 1, value: mc6809_os9_data8(flags), kind: Addr8
# CHECK: lda #mc6809_os9_data8(flags)     ; encoding: [0x86,A]
# CHECK-NEXT: fixup A - offset: 1, value: mc6809_os9_data8(flags), kind: Addr8

# RELOC:      Section ({{[0-9]+}}) .rela.text {
# RELOC-NEXT:   0x2 R_MC6809_OS9_OFFSET_16 counter 0x0
# RELOC-NEXT:   0x5 R_MC6809_OS9_OFFSET_8 counter 0x0
# RELOC-NEXT:   0x7 R_MC6809_OS9_OFFSET_8 flags 0x0
# RELOC-NEXT:   0x9 R_MC6809_OS9_OFFSET_8 flags 0x0
# RELOC-NEXT: }

        .section .dp.bss,"aw",@nobits
        .globl counter, flags
counter:
        .zero 2
        .section .dp.data,"aw",@progbits
flags:
        .byte 3
