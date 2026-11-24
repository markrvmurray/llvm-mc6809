; Every CIE names the PC as the return-address column: the return address
; is the PC a call pushed, and the FDEs say where it went.  With no register
; named, every CIE said column -1 -- in .debug_frame a column no consumer can
; follow, and in .eh_frame, which stores the column in one byte, an assertion
; in the assembler.  PC is DWARF register 5.
;
; RUN: llvm-mc -triple=mc6809 --filetype=obj -o %t.o %s
; RUN: llvm-dwarfdump --debug-frame %t.o | FileCheck %s
; RUN: llvm-mc -triple=mc6809 --filetype=obj --defsym=EH=1 -o %t.eh.o %s
; RUN: llvm-dwarfdump --eh-frame %t.eh.o | FileCheck %s

.ifndef EH
	.cfi_sections .debug_frame
.endif
	.text
f:
	.cfi_startproc
	.cfi_def_cfa s, 2
	.cfi_offset pc, -2
	pshs	u
	.cfi_adjust_cfa_offset 2
	.cfi_offset u, -4
	puls	u,pc
	.cfi_endproc

; CHECK: CIE
; CHECK: Return address column: 5
