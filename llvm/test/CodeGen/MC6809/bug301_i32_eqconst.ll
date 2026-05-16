; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301 (2026-05-16): the two "is X equal-to-K /
; not-equal-to-K" predicates at i32 width for non-zero constant K must
; lower via the EqConst_i32 pseudo (which post-RA-expands to SUBW #lo +
; SBCD #hi + CC.Z extraction) NOT via the __cmpsi2 libcall.

; CHECK-LABEL: i32_eq_42:
define i1 @i32_eq_42(i32 %x) {
  ; SUBW #42 (low half, 0x2A) ; SBCD #0 (high half) ; CC.Z extraction.
  ; CHECK: subw #42
  ; CHECK: sbcd #0
  ; CHECK: tfr cc,a
  ; CHECK: anda #4
  ; CHECK: lsra
  ; CHECK: lsra
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp eq i32 %x, 42
  ret i1 %cmp
}

; CHECK-LABEL: i32_ne_42:
define i1 @i32_ne_42(i32 %x) {
  ; NE = NOT EQ — EqConst_i32 + EORB #1.
  ; CHECK: subw #42
  ; CHECK: sbcd #0
  ; CHECK: tfr cc,a
  ; CHECK: anda #4
  ; CHECK: lsra
  ; CHECK: lsra
  ; CHECK: eorb #1
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp ne i32 %x, 42
  ret i1 %cmp
}

; CHECK-LABEL: i32_eq_big:
define i1 @i32_eq_big(i32 %x) {
  ; Large constant 0x12345678 splits to SUBW #0x5678 + SBCD #0x1234.
  ; CHECK: subw #22136
  ; CHECK: sbcd #4660
  ; CHECK: tfr cc,a
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp eq i32 %x, 305419896
  ret i1 %cmp
}

; CHECK-LABEL: i32_eq_neg_one:
define i1 @i32_eq_neg_one(i32 %x) {
  ; Negative constant -1 = 0xFFFFFFFF splits to SUBW #0xFFFF + SBCD #0xFFFF.
  ; CHECK: subw #65535
  ; CHECK: sbcd #65535
  ; CHECK: tfr cc,a
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp eq i32 %x, -1
  ret i1 %cmp
}
