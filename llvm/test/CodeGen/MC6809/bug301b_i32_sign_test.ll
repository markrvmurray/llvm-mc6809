; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301b (2026-05-16): the four "is X negative / non-
; negative" predicates at i32 width must lower via the SignTest_i32
; pseudo (which post-RA-expands to ASLB + LDB #0 + ADCB #0 — capturing
; the sign bit via CC.C) NOT via the __cmpsi2 libcall.
;
; The libcall path emitted a trunc-then-eq sequence whose post-RA
; spill placer reused the slot holding cmpsi2's result for the
; comparand `rem` before the trunc consumed the result, dropping the
; sign bit silently for inputs where rem.lo's low byte happened to be 0
; (~1/256 of inputs, manifest in gmtime hour-+24 for 5 specific
; time_t values).
;
; The four predicate shapes are:
;   icmp slt X, 0       canonical "X < 0"
;   icmp sge X, 0       canonical "X >= 0"
;   icmp sgt X, -1      optimizer-normalised "X >= 0"
;   icmp sle X, -1      optimizer-normalised "X < 0"
;
; CHECK-LABEL: i32_slt_zero:
define i1 @i32_slt_zero(i32 %x) {
  ; The SignTest_i32 expansion: ASLB + LDB #0 + ADCB #0.  No __cmpsi2.
  ; Bug #304 followup (2026-05-21): SignTest_i32 now works in AA
  ; (no longer TFR A,B first) to save the initial-TFR overhead.
  ; CHECK: asla
  ; CHECK: lda #0
  ; CHECK: adca #0
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp slt i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_sge_zero:
define i1 @i32_sge_zero(i32 %x) {
  ; sge X, 0 = NOT (slt X, 0).  Same SignTest_i32 + XOR with 1.
  ; Bug #304 followup (2026-05-21): SignTest_i32 now works in AA
  ; (no longer TFR A,B first) to save the initial-TFR overhead.
  ; CHECK: asla
  ; CHECK: lda #0
  ; CHECK: adca #0
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sge i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_sgt_minus_one:
define i1 @i32_sgt_minus_one(i32 %x) {
  ; Optimizer-normalised "X >= 0".  Same fast path.
  ; Bug #304 followup (2026-05-21): SignTest_i32 now works in AA
  ; (no longer TFR A,B first) to save the initial-TFR overhead.
  ; CHECK: asla
  ; CHECK: lda #0
  ; CHECK: adca #0
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sgt i32 %x, -1
  ret i1 %cmp
}

; CHECK-LABEL: i32_sle_minus_one:
define i1 @i32_sle_minus_one(i32 %x) {
  ; Optimizer-normalised "X < 0".  Same fast path.
  ; Bug #304 followup (2026-05-21): SignTest_i32 now works in AA
  ; (no longer TFR A,B first) to save the initial-TFR overhead.
  ; CHECK: asla
  ; CHECK: lda #0
  ; CHECK: adca #0
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sle i32 %x, -1
  ret i1 %cmp
}

; CHECK-LABEL: i32_slt_reg_falls_through:
define i1 @i32_slt_reg_falls_through(i32 %x, i32 %y) {
  ; Register-RHS i32 ICMP (not constant) — falls through to __cmpsi2
  ; until Compare_i32_Reg / Compare_i32_Mem land in the register-RHS
  ; and memory-RHS sub-steps.  Proves the sign-test fast path AND
  ; the constant-RHS compare-and-set / compare-and-branch fast paths
  ; don't over-trigger.
  ; CHECK: lbsr __cmpsi2
  %cmp = icmp slt i32 %x, %y
  ret i1 %cmp
}
