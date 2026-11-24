; A PC-relative data word -- `.word target - .`, or an .eh_frame FDE's start
; address, which is encoded PC-relative -- becomes a PC-relative relocation
; when its target is in another section.  It was written as an absolute one,
; so after linking it held the target's address where the distance belonged:
; every .eh_frame FDE named the wrong code.  A 4-byte PC-relative word has no
; relocation on this target, and is an error rather than a wrong answer.
;
; RUN: llvm-mc -triple=mc6809 -filetype=obj -o %t.o %s
; RUN: llvm-readelf -r %t.o | FileCheck %s --check-prefix=REL
; RUN: ld.lld --image-base=0 -Ttext=0x1000 --section-start=.rodata=0x2000 -e f %t.o -o %t.elf
; RUN: llvm-dwarfdump --eh-frame %t.elf | FileCheck %s --check-prefix=EH
; RUN: llvm-objdump -s -j .rodata %t.elf | FileCheck %s --check-prefix=WORD
; RUN: not llvm-mc -triple=mc6809 -filetype=obj --defsym=LONG=1 -o /dev/null %s 2>&1 | FileCheck %s --check-prefix=ERR

	.text
	.globl	f
f:
	.cfi_startproc
	pshs	x
	.cfi_adjust_cfa_offset 2
	puls	x,pc
	.cfi_endproc

	.section .rodata,"a",@progbits
distance:
	.word	f - .
.ifdef LONG
	.long	f - .
.endif

; R_MC6809_PCREL_16 counts from the end of its field, as a branch does from
; the end of its instruction; a data word counts from its start, two bytes
; earlier, which the addend says.
; REL: R_MC6809_PCREL_16 {{.*}} f + 2
; REL: R_MC6809_PCREL_16 {{.*}} .text + 2
; REL-NOT: R_MC6809_FK_DATA_4

; The FDE covers f, at $1000, four bytes long.
; EH: FDE cie={{[0-9a-f]+}} pc=00001000...00001004

; The word at $2000 holds f's distance from itself, $1000 - $2000, not f's
; address.
; WORD: 2000 f000

; ERR: error: {{.*}}4-byte PC-relative
