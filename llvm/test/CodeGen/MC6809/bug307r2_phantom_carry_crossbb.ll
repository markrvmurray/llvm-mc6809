; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #307 round 2 sentinel — cross-BB phantom_carry bridging.
;
; `selectAddE` / `selectSubE` (10 emit sites for imm-RHS, G_UNMERGE-fold,
; mem-RHS, reg-RHS — both add and sub) previously emitted
; `.addUse(CarryIn, RegState::Implicit)` on the resulting
; AddSetCarryUse / SubSetCarryUse pseudo, regardless of whether the
; `ensureCarryChainIntegrity` bridge had fired.
;
; When the bridge fires (cross-BB or across a CC-clobbering MI), the
; implicit-use of the phantom_carry vreg forces regalloc to keep it
; live across the bridge.  Across a call that's impossible — phantom
; physregs are caller-clobbered and have no spill storage — so the
; MachineVerifier flagged "Using an undefined physical register
; $phantom_carry_X" at `MaterializeCC_C_to_byte`.
;
; Fix: `ensureCarryChainIntegrity` now returns `bool`; selectAddE /
; selectSubE only attach `.addUse(CarryIn, RegState::Implicit)` when
; the bridge did NOT fire.
;
; Manifest: imaxabs / llabs at -Og-hd6309-mame.  The cross-BB shape
; below mirrors `llabs(i64)` from picolibc.
;
; Compilation must succeed with -verify-machineinstrs.

define i64 @bug307r2_llabs(i64 %x) {
; CHECK-LABEL: bug307r2_llabs:
entry:
  %lt = icmp slt i64 %x, 0
  br i1 %lt, label %neg, label %pos

neg:
  %n = sub nsw i64 0, %x
  br label %exit

pos:
  br label %exit

exit:
  %r = phi i64 [ %n, %neg ], [ %x, %pos ]
  ret i64 %r
}

; CHECK: rts
