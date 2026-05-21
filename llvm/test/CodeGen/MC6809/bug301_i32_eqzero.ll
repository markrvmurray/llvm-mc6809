; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301 (2026-05-16): the two "is X zero / non-zero"
; predicates at i32 width must lower via the EqZero_i32 pseudo (which
; post-RA-expands to an ORR-chain or LDB+3xORB chain + CC.Z extraction)
; NOT via the __cmpsi2 libcall.

; CHECK-LABEL: i32_eq_zero:
define i1 @i32_eq_zero(i32 %x) {
  ; The EqZero_i32 expansion: 4-byte OR-chain followed by tfr cc,a /
  ; anda #4 / lsra / lsra to materialise CC.Z as a 0/1 byte.
  ;
  ; Bug #308 (2026-05-19) sharpening: with the Q-spill source path now
  ; using LDQ-via-$aq + AQ save/restore (instead of the old Bug #221
  ; two-LDD-via-$ad path), the EqZero_i32 expansion now lands the i32
  ; in $aq via LDQ and does the OR-chain as ORR a,b / ORR e,b / ORR
  ; f,b (sub-byte register-to-register ORs).  This is a strict
  ; codegen win over the old 4-byte-load + 3-byte-OR chain through
  ; memory.
  ;
  ; Bug #304 (2026-05-21) followup: EqZero_i32 now declares AQ in
  ; its Defs (mirroring SignTest_i32's fix).  The regalloc emits a
  ; U-frame-pointer save+materialise of the i32 source now that AQ
  ; is known to be clobbered.  The LDQ source can be either S- or
  ; U-relative depending on regalloc — accept either with [su].
  ; CHECK: ldq {{[0-9]+}},{{[su]}}
  ; Bug #304 followup (2026-05-21): EqZero_i32 now ORs into AA
  ; (`orr b,a; orr e,a; orr f,a`) instead of AB, saving the final
  ; TFR A,B that brought the extracted Z bit into AB.
  ; CHECK: orr b,a
  ; CHECK: orr e,a
  ; CHECK: orr f,a
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
  ; Same Bug #308 LDQ-via-$aq sharpening as i32_eq_zero above.
  ; Same Bug #304 followup (S- or U-relative LDQ).
  ; CHECK: ldq {{[0-9]+}},{{[su]}}
  ; Bug #304 followup (2026-05-21): EqZero_i32 now ORs into AA
  ; (`orr b,a; orr e,a; orr f,a`) instead of AB, saving the final
  ; TFR A,B that brought the extracted Z bit into AB.
  ; CHECK: orr b,a
  ; CHECK: orr e,a
  ; CHECK: orr f,a
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
