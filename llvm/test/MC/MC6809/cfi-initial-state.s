; Every CIE carries the rule a call leaves at entry: CFA = S + 2, the return
; address (PC) at CFA - 2.  A hand-written routine's CFI then needs only to
; follow its own pushes and pulls; without the initial rule its first
; .cfi_adjust_cfa_offset had no CFA to adjust, and the FDE was unreadable.
;
; RUN: llvm-mc -triple=mc6809 --filetype=obj -o %t.o %s
; RUN: llvm-dwarfdump --debug-frame %t.o | FileCheck %s

	.cfi_sections .debug_frame
	.text
f:
	.cfi_startproc
	pshs	x
	.cfi_adjust_cfa_offset 2
	puls	x,pc
	.cfi_endproc

; CHECK:      CIE
; CHECK:      DW_CFA_def_cfa: SS +2
; CHECK-NEXT: DW_CFA_offset: PC -2
; CHECK:      FDE
; CHECK:      0x0: CFA=SS+2: PC=[CFA-2]
; CHECK-NEXT: 0x2: CFA=SS+4: PC=[CFA-2]
; CHECK-NOT:  error
