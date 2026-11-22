; CFI directives take MC6809 register names, and an unknown one is an error.
;
; The assembler's register hook used to read its start and end locations as
; the text to match, when the directive parser passes them to be filled in,
; so a named register matched nothing and the directive was dropped without
; a word.  Register numbers were unaffected.
;
; RUN: llvm-mc -triple=mc6809 --filetype=obj -o %t.o %s
; RUN: llvm-dwarfdump --debug-frame %t.o | FileCheck %s
; RUN: not llvm-mc -triple=mc6809 --filetype=obj --defsym=BAD=1 -o /dev/null \
; RUN:   %s 2>&1 | FileCheck --check-prefix=ERR %s

	.cfi_sections .debug_frame
	.text
named:
	.cfi_startproc
	.cfi_undefined pc
	.cfi_offset x, -4
	.cfi_offset Y, -6
	.cfi_offset u, -8
	.cfi_same_value s
	.cfi_offset d, -10
	.cfi_offset a, -11
	.cfi_offset b, -12
	.cfi_offset cc, -13
	.cfi_offset dp, -14
	.cfi_undefined 5
	rts
	.cfi_endproc

; CHECK:      DW_CFA_undefined: PC
; CHECK-NEXT: DW_CFA_offset: IX -4
; CHECK-NEXT: DW_CFA_offset: IY -6
; CHECK-NEXT: DW_CFA_offset: SU -8
; CHECK-NEXT: DW_CFA_same_value: SS
; CHECK-NEXT: DW_CFA_offset: AD -10
; CHECK-NEXT: DW_CFA_offset: AA -11
; CHECK-NEXT: DW_CFA_offset: AB -12
; CHECK-NEXT: DW_CFA_offset: CC -13
; CHECK-NEXT: DW_CFA_offset: DP -14
; CHECK-NEXT: DW_CFA_undefined: PC

.ifdef BAD
	.cfi_startproc
	.cfi_undefined nosuchreg
	rts
	.cfi_endproc
.endif

; ERR: error: invalid register name
; ERR-NEXT: .cfi_undefined nosuchreg
