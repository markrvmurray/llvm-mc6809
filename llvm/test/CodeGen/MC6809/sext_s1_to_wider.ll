; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:     -stop-after=legalizer < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 \
; RUN:     -stop-after=legalizer < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 \
; RUN:     -stop-after=legalizer < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 \
; RUN:     -stop-after=legalizer < %s | FileCheck %s
;
; G_SEXT (s16, s1) and (s32, s1) legalization regression test.
;
; The MC6809 G_SEXT rule custom-lowers s1-source extends using the
; MOS-proven select-on-constants pattern (MOSLegalizerInfo.cpp:596):
;   select s1 cond, -1, 0
; at the destination width. This completely avoids chained sext chains
; and the upstream lowerEXT MERGE_VALUES(src, ASHR_HI) pattern, keeping
; register pressure at just 1 i16 (D) at any point in the expansion.
; The select then lowers via MC6809LowerSelect into a branch +
; constant-materialization diamond.
;
; This test verifies the legalize output has G_SELECT, not G_SEXT chains.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: name: sext_s1_to_s16
; The legalizer must produce a select-on-constants, not a chained sext.
; CHECK: G_SELECT
; CHECK-NOT: G_SEXT {{.*}}(s16){{.*}}(s1)
define i16 @sext_s1_to_s16(i16 %x) {
  %is_zero = icmp eq i16 %x, 0
  %ext = sext i1 %is_zero to i16
  ret i16 %ext
}

; CHECK-LABEL: name: sext_s1_to_s32
; Same: select-on-constants at s32 width.
; CHECK: G_SELECT
; CHECK-NOT: G_SEXT {{.*}}(s32){{.*}}(s1)
define i32 @sext_s1_to_s32(i16 %x) {
  %is_zero = icmp eq i16 %x, 0
  %ext = sext i1 %is_zero to i32
  ret i32 %ext
}
