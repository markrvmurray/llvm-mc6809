# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o
# RUN: llvm-objdump -d %t.o | FileCheck %s
# Bytes at the end of a section that do not form a whole instruction (a jump
# table, padding) are skipped a byte at a time: the disassembler reports a
# size on failure rather than leaving the caller's unset, which llvm-objdump
# slices the section by.
	.text
	.globl f
f:
	cmpa $ffb1
	.byte 0xff, 0x83
# CHECK: cmpa $ffb1
# CHECK-NEXT: ff <unknown>
# CHECK-NEXT: 83 <unknown>
