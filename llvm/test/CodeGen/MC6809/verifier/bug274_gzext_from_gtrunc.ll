; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #274 (Og hit at picolibc test-ctype.c main, line 169:37):
; the `bool punct = print && !space && !alnum;` chain emits
;   %1562:s8 = G_ANYEXT %81:s1
;   %1563:s8 = G_XOR %1562, -1            ; !space at i8 width
;   %85:s1   = G_TRUNC %1563
;   %1643:s8 = G_ZEXT %85
; G_TRUNC s8→s1 selects to AND_i8_Imm tied (line 1004 of
; MC6809InstructionSelector.cpp), which forces %85 into ACC8 class.
; G_ZEXT s1→s8 then matches the TableGen pattern (ZEX8Implicit, which
; expects ABLSBc input) — constrain demotes silently and
; -verify-machineinstrs flags the class mismatch.
;
; Fix: G_ZEXT s1→s8's hand-selector detects when SrcReg's defining MI
; is G_TRUNC s8→s1 and emits AND_i8_Imm tied directly. The second AND
; #1 is idempotent against an already-masked input, so codegen is
; correct and the verifier sees consistent classes throughout.

; CHECK-LABEL: probe_zext_from_trunc:
; CHECK-NOT: Bad machine code

define i8 @probe_zext_from_trunc(i8 %a, i1 %space) {
entry:
  ; Mimic the post-legalisation shape: anyext s1→s8, XOR -1, trunc, zext.
  %not_space = xor i1 %space, true
  %r = zext i1 %not_space to i8
  ret i8 %r
}
