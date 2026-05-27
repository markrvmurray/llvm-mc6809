; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -debug-pass=Structure < %s -o /dev/null 2>&1 | FileCheck %s
;
; Bug #360 follow-up (cmpx/cmpy/tst compare-zero elision) regression guard.
;
; MC6809LateOptimization's elideCompareZero drops a redundant CMP/TST #0 by
; letting the conditional branch read the *producer's* flags directly. That
; is only sound on the FINAL instruction stream: run before FinalLowering,
; the elision removes the CMP and FinalLowering's store-reload / LEA-pointer-
; spill classes then delete the very (redundant-reload) instruction whose
; flags the branch now depends on, leaving the branch reading an adjacent
; LEAX's hardware Z (LEAX sets Z on hardware but carries no MIR flag def).
; That mis-elided strrchr's `ldy slot; cmpy #0; bne` null-check into an
; infinite loop (strchr called on a garbage pointer).
;
; The fix is purely pass placement: MC6809LateOptimization must run AFTER
; MC6809FinalLowering and BEFORE BranchRelaxation. This test pins that order
; in the codegen pipeline so a move back to addPreSched2 (before
; FinalLowering) is caught immediately. See
; MC6809TargetMachine.cpp::addPreEmitPass.
;
; CHECK: -mc6809-final-lowering
; CHECK-SAME: -mc6809-late-opt
; CHECK-SAME: -branch-relaxation

define void @f(ptr %p) {
  %c = icmp eq ptr %p, null
  br i1 %c, label %a, label %b
a:
  ret void
b:
  ret void
}
