; RUN: llc -mtriple=mc6809 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #281 Shape A (closed 2026-05-12):
; A G_PHI producing an i1 vreg whose register class is forced to ACC8
; by downstream class propagation, feeding G_ZEXT i1→i8, would
; select to `%dst:abc = ZEX8Implicit %src:acc8`. The imported
; ZEX8Implicit pattern's InOperandList is ABLSBc (a 1-bit sub-reg
; class), disjoint from ACC8 on the class hierarchy. The
; constrain-machinery cannot bridge the gap, leaving the MIR with
; %src:acc8 against an ABLSBc operand expectation:
;
;   *** Bad machine code: Illegal virtual register for instruction ***
;   - instruction: %X:abc = ZEX8Implicit %Y:acc8
;   Expected a ABLSBc register, but got a ACC8 register
;
; Bug #274's commit f169d4094d86 added a detector for this scenario
; when the source is the output of G_TRUNC s8→s1 (which gets selected
; to AND_i8_Imm tied, forcing ACC8). That detector walks
; MRI->getVRegDef(SrcReg) and matches G_TRUNC explicitly. Bug #281
; surfaces the same issue but with a G_PHI producer — the
; G_TRUNC-only detector misses it.
;
; Fix: broaden the detector to a class-based check. When SrcReg's
; class is in the ACC8 hierarchy and NOT in the BIT1 hierarchy,
; replace G_ZEXT with AND_i8_Imm tied (idempotent on i1-bounded
; byte-wide sources).
;
; Closes the 15 of 21 Og-fp residual hits at test-math TUs
; (test-cacos.c, test-casin.c, test-pow.c, test-atan2.c, test-hypot.c,
; etc. — all include test-complex-one.h which carries the failing
; PHI shape).

; CHECK-LABEL: phi_i1_then_zext_to_i8:
; CHECK-NOT: Bad machine code

define i8 @phi_i1_then_zext_to_i8(i1 %cond, i1 %a, i1 %b) {
entry:
  br i1 %cond, label %then, label %else

then:
  br label %merge

else:
  br label %merge

merge:
  ; The PHI's output class gets forced to ACC8 by downstream
  ; consumer constraints (the zext below) in the failing scenario.
  ; Verifier flags the ZEX8Implicit class mismatch without the fix.
  %v = phi i1 [ %a, %then ], [ %b, %else ]
  %r = zext i1 %v to i8
  ret i8 %r
}
