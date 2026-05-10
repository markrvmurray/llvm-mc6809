; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s | FileCheck %s
;
; Bug #271 category 5 sentinel: post-RA expansion of fused
; CompareBranch / TestBranch pseudos used to lose the implicit
; CFG-fallthrough edge. The pseudo had two CFG successors (the
; conditional target + the implicit fallthrough); the expansion to
; Compare + LBlbc only handles the conditional target. When the
; CFG-fallthrough was a different MBB than layout-next, the BB ended
; up with a CFG successor that no terminator reached — verifier
; flagged it as "MBB has unexpected successors".
;
; Concrete sites: __ubsan_val_to_imax / __ubsan_val_to_umax in
; libc/ubsan/ at -Og hd6309 (switch dispatch from bb.0).
;
; The fix in expandFusedCompareBranch (MC6809InstrInfo.cpp ~5821)
; emits a LongBranchRelative to the CFG-fallthrough when:
;   (a) the fused pseudo is the LAST terminator in its BB
;       (otherwise insertBranch already placed an unconditional
;       branch — adding another would duplicate, e.g. loop.ll), AND
;   (b) the CFG-fallthrough != layout-next.
;
; This .ll exercises a switch-on-i16 where the entry block dispatches
; via a fused CompareBranch and the fallthrough successor isn't the
; layout-next BB.

define i16 @bug271_cat5(i16 %width, i16 %def) {
; CHECK-LABEL: bug271_cat5:
entry:
  switch i16 %width, label %ret_def [
    i16 8,  label %case8
    i16 16, label %case16
    i16 32, label %case32
    i16 64, label %case64
  ]

case8:
  ret i16 1

case16:
  ret i16 2

case32:
  ret i16 3

case64:
  ret i16 4

ret_def:
  ret i16 %def
}
