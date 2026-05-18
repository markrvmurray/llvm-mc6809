; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #308 sentinel — Q-spill slot-to-slot copies must use LDQ-via-AQ
; with conditional save/restore, NOT the older two-LDD-via-AD path.
;
; The two-LDD path called `emitTwoLDDSlotCopy`, which transiently wrote
; $ad in the middle of i32 arithmetic.  When AD-family registers were
; live (e.g. $aw still holding the high half of a different i32), the
; second LDD silently clobbered them and the MachineVerifier rejected
; the function ("Using an undefined physical register $aw").
;
; Fix: `expandLoadIdx` / `expandStoreIdx`'s SPILL_Q* path now emits a
; single LDQ-via-AQ wrapped by `emitAQPreservedOverHardStackScratch`
; (LEAS-based save/restore of $aq) when AD-family liveness is detected.
;
; Manifest: imaxabs / llabs at -Og-hd6309-mame produced verifier hits.
; Compilation must succeed with -verify-machineinstrs clean.

define i64 @bug308_imaxabs(i64 %x) {
; CHECK-LABEL: bug308_imaxabs:
entry:
  %lt = icmp slt i64 %x, 0
  %neg = sub nsw i64 0, %x
  %r = select i1 %lt, i64 %neg, i64 %x
  ret i64 %r
}

; CHECK: rts
