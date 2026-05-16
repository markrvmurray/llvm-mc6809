; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301 Phase D step 1 (2026-05-16): the two "is X zero / non-zero"
; predicates at i32 width must lower via the EqZero_i32 pseudo (which
; post-RA-expands to an ORR-chain or LDB+3xORB chain + CC.Z extraction)
; NOT via the __cmpsi2 libcall.

; CHECK-LABEL: i32_eq_zero:
define i1 @i32_eq_zero(i32 %x) {
  ; The EqZero_i32 expansion for SPILL_Q-source: ldb + 3x orb (4-byte
  ; OR-chain) followed by tfr cc,a / anda #4 / lsra / lsra to
  ; materialise CC.Z as a 0/1 byte.
  ; CHECK: ldb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: tfr cc,a
  ; CHECK: anda #4
  ; CHECK: lsra
  ; CHECK: lsra
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp eq i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_ne_zero:
define i1 @i32_ne_zero(i32 %x) {
  ; NE = NOT EQ — EqZero_i32 + XOR with 1 (encoded as eorb #1).
  ; CHECK: ldb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: orb {{[0-9]+}},u
  ; CHECK: tfr cc,a
  ; CHECK: anda #4
  ; CHECK: lsra
  ; CHECK: lsra
  ; CHECK: eorb #1
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp ne i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_slt_reg_falls_through:
define i1 @i32_slt_reg_falls_through(i32 %x, i32 %y) {
  ; Register-RHS i32 ICMP (not constant) — falls through to __cmpsi2
  ; until Compare_i32_Reg / Compare_i32_Mem land in the register-RHS
  ; and memory-RHS sub-steps.  Proves the constant-RHS fast paths
  ; don't over-trigger.
  ; CHECK: lbsr __cmpsi2
  %cmp = icmp slt i32 %x, %y
  ret i1 %cmp
}
