; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301: signed/unsigned i32 ICMP against constant RHS with a
; non-brcond consumer (i1 stored, returned, or chained through
; boolean logic) must lower via the CompareSet_i8_i32_Imm fused
; pseudo (post-RA-expanding to SUBW + SBCD + branch-free CC bit
; extraction) NOT via the __cmpsi2 libcall.
;
; (The fused compare-and-branch pseudo handles the brcond consumer
; case in a separate code path.  This test covers the materialize-
; at-use path.)

; CHECK-LABEL: i32_slt_store_i1:
define void @i32_slt_store_i1(i32 %x, ptr %p) {
  ; SLT against constant — predicate needs N XOR V extracted.
  ; CHECK: subw #42
  ; CHECK: sbcd #0
  ; CHECK: tfr cc,a
  ; CHECK: anda #10
  ; CHECK: lsra
  ; CHECK: lsrb
  ; CHECK: lsrb
  ; CHECK: eorr b,a
  ; CHECK: anda #1
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp slt i32 %x, 42
  %byte = zext i1 %cmp to i8
  store i8 %byte, ptr %p
  ret void
}

; CHECK-LABEL: i32_ult_return_i1:
define i1 @i32_ult_return_i1(i32 %x) {
  ; ULT against constant — predicate needs just C bit (CC bit 0).
  ; CHECK: subw #1000
  ; CHECK: sbcd #0
  ; CHECK: tfr cc,a
  ; CHECK: anda #1
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp ult i32 %x, 1000
  ret i1 %cmp
}

; CHECK-LABEL: i32_sgt_return_i1:
define i1 @i32_sgt_return_i1(i32 %x) {
  ; SGT against constant — predicate needs ~(Z | (N XOR V)).
  ; This is the most expensive path: ~12 instructions for the
  ; CC extraction.  Still beats the __cmpsi2 LBSR + spill in cycles.
  ; CHECK: subw
  ; CHECK: sbcd
  ; CHECK: tfr cc,a
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sgt i32 %x, 42
  ret i1 %cmp
}

; CHECK-LABEL: i32_slt_xor_chain:
define i1 @i32_slt_xor_chain(i32 %x, i1 %z) {
  ; Predicate fed into boolean XOR (not brcond, not select).
  ; Materialize the i1 via CompareSet_i8_i32_Imm then XOR.
  ; CHECK: subw #42
  ; CHECK: sbcd #0
  ; CHECK: tfr cc,a
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp slt i32 %x, 42
  %res = xor i1 %cmp, %z
  ret i1 %res
}
