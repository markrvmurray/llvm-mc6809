; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Bug #268 sentinel: indirect call through a function pointer must
; reach JSRi_o0 with the callee operand constrained to INDEX16.
; Pre-fix, MC6809CallLowering passed Info.Callee through to JSRi_o0
; with .add(), leaving the callee vreg without a register class —
; -verify-machineinstrs flagged it as "Expect register class INDEX16
; but got nothing".
;
; Fix: in MC6809CallLowering::lowerCall, when emitting an indirect
; JSRi_o0, pre-constrain the callee vreg to MC6809::INDEX16RegClass
; via MRI.setRegClass (only when the vreg has no class — for already-
; classed vregs the existing class is left alone).

define i16 @bug268(ptr %fp, i16 %a, i16 %b) {
; CHECK-LABEL: bug268:
; CHECK: jsr ,
  %r = call i16 %fp(i16 %a, i16 %b)
  ret i16 %r
}
