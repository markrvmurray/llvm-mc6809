; lldb reads an MC6809 ELF's symbol table at the ELF32 layout.
;
; MC6809 addresses are 16 bits, and lldb reads the DWARF in an MC6809
; object with 2-byte addresses.  The ELF structures are another matter: a
; symbol table entry is laid out by the ELF class, and lldb's parser picks
; the layout from the reader's address size.  Reading .symtab with the
; 2-byte size took every 16-byte ELF32 symbol as a 24-byte ELF64 one: the
; symbols came out Invalid with shifted fields, and one made-up symbol
; covered the whole text section, so the unwinder stopped at the first
; frame.
;
; REQUIRES: lldb
; RUN: llvm-mc -triple=mc6809 --filetype=obj -o %t.o %s
; RUN: ld.lld --image-base=0 -Ttext=0x4000 -e first %t.o -o %t.elf
; RUN: %lldb -b -o "target create %t.elf" -o "image dump symtab" \
; RUN:   -o "image lookup -a 0x4005" 2>&1 | FileCheck %s

; CHECK-NOT: Invalid
; CHECK-DAG: Code {{.*}}0x0000000000004000 {{.*}}0x0000000000000004 {{.*}}first
; CHECK-DAG: Code {{.*}}0x0000000000004004 {{.*}}0x0000000000000003 {{.*}}second
; CHECK-DAG: Data {{.*}}value
; CHECK-NOT: ___lldb_unnamed_symbol
; CHECK: Summary: {{.*}}`second + 1

	.text
	.globl	first
	.type	first,@function
first:
	ldd	#0x1234
	rts
	.size	first, .-first

	.globl	second
	.type	second,@function
second:
	clra
	clrb
	rts
	.size	second, .-second

	.data
	.globl	value
	.type	value,@object
value:
	.word	0x5678
	.size	value, 2
