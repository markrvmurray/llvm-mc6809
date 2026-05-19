; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #307 round 2 follow-up sentinel — narrow implicit-use drop gate
; to cross-BB bridges only.
;
; The initial Bug #307 round 2 fix changed
; `ensureCarryChainIntegrity`'s signature `void → bool` and made
; ALL bridges (cross-BB AND same-BB CC-clobber) signal "drop the
; phantom_carry implicit-use".  But the same-BB case keeps the
; phantom_carry vreg local — DCE protection is still needed
; because the producer's non-carry byte result may be dead.
;
; Narrowed: only cross-BB bridges signal the drop
; (`return CrossBB` instead of `return true`).  Same-BB CC-clobber
; bridges retain `.addUse(CarryIn, RegState::Implicit)` for DCE
; protection — its original Bug #161 round 12 purpose.
;
; This sentinel exercises a same-BB i64 carry chain — compilation
; must succeed with -verify-machineinstrs clean, demonstrating that
; the narrowed gate handles same-BB shapes via the original
; implicit-use route.

define i64 @bug307r2_samebb_chain(i64 %a, i64 %b, i64 %c) {
; CHECK-LABEL: bug307r2_samebb_chain:
entry:
  %t1 = add i64 %a, %b
  %t2 = add i64 %t1, %c
  %t3 = sub i64 %t2, %a
  ret i64 %t3
}

; CHECK: rts
