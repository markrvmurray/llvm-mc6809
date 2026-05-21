; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s
;
; Phase 3.3 verification: with G_LOAD s32 legal natively (Phase 3.2),
; an i32 G_ADD reading its memory operand should produce LDQ + ADDW +
; ADCD (not 2× LDD + 4× byte adds).  This is the "carry chains still
; apply post-flip" check from the plan: expandAddSub_i32_Mem still
; works when source operands come from native LDQ.
;
; Specifically: %a = load i32; %s = add i32 %a, %b should produce a
; tight LDQ + ADDW/ADCD pair on HD6309.  Pre-Phase-3 (when G_LOAD s32
; narrowed to byte chain), this would expand to 4 byte adds via the
; legacy decomposition path.

target triple = "mc6809-unknown-unknown"

define i32 @add_load_reg(i32* %p, i32 %b) {
  %a = load i32, i32* %p
  %s = add i32 %a, %b
  ret i32 %s
}

; CHECK-LABEL: add_load_reg:
; CHECK:       ldq
; CHECK:       addw
; CHECK-NEXT:  adcd

define i32 @add_load_imm(i32* %p) {
  %a = load i32, i32* %p
  %s = add i32 %a, 305419896
  ret i32 %s
}

; CHECK-LABEL: add_load_imm:
; CHECK:       ldq
; CHECK:       addw
; CHECK-NEXT:  adcd

define i32 @sub_load_reg(i32* %p, i32 %b) {
  %a = load i32, i32* %p
  %s = sub i32 %a, %b
  ret i32 %s
}

; CHECK-LABEL: sub_load_reg:
; CHECK:       ldq
; CHECK:       subw
; CHECK-NEXT:  sbcd
