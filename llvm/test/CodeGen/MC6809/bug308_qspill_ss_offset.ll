; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #308 follow-up sentinel — $ss-offset compensation inside the
; LEAS-based AQ-save/restore wrapper.
;
; `emitAQPreservedOverHardStackScratch` emits `LEAS -4,$ss` before the
; body and `LEAS 4,$ss` after.  When the wrapped body's source or
; destination index register is $ss itself, the in-body slot offset
; must be increased by 4 to compensate for the prologue's `LEAS -4,$ss`
; — otherwise the slot copy reads/writes the wrong stack address.
;
; Symptom: i64 functions with $ss-relative spill slots and AD-family
; liveness produced incorrect runtime results when the compensation
; was missing.
;
; Compilation must succeed with -verify-machineinstrs.  The runtime
; consequence (returned value depends on $ss-relative slot integrity)
; is exercised by picolibc's imaxabs / llabs tests.

define i64 @bug308_ss_offset_smoke(i64 %x, i64 %y, i64 %z) {
; CHECK-LABEL: bug308_ss_offset_smoke:
entry:
  %a = add i64 %x, %y
  %b = sub i64 %a, %z
  %neg_b = sub nsw i64 0, %b
  %lt = icmp slt i64 %b, 0
  %r = select i1 %lt, i64 %neg_b, i64 %b
  ret i64 %r
}

; CHECK: rts
