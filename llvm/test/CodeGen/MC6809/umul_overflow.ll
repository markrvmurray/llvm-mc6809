; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
;
; G_UMULO legalization regression test for bug #83.
;
; clang lowers __builtin_umul_overflow to llvm.umul.with.overflow.iN,
; which becomes G_UMULO in GMIR. Before bug #83's fix the MC6809 rule
; for G_UMULO was `.libcall()`, but LegalizerHelper::libcall() has no
; G_UMULO case, so the action was Libcall but the dispatch returned
; UnableToLegalize. The existing custom handler legalizeMultiplyWith-
; Overflow was therefore unreachable dead code.
;
; The fix changes the rule to .customForCartesianProduct(...) so the
; existing handler fires. The handler does:
;   result   = LHS * RHS                          (G_MUL — libcalls for s32)
;   overflow = LHS > UINT_MAX / max(RHS, 1)       (G_UDIV libcall + G_UMAX lower)
;
; The fix covers s8/s16/s32. s64 is NOT covered yet — the custom handler
; produces s64 vreg intermediates that MC6809RegisterBankInfo cannot
; classify (only up to 32 bits today). The s64 path is part of the
; broader long-long (i64) roadmap.
;
; The functions below mirror the picolibc strto* family pattern:
; extract the result and the overflow flag separately, use them, and
; return a saturating value. They do NOT return the {iN, i1} aggregate
; (returning small aggregates is a separate backend gap).
;
; This test only checks that the functions compile (no instruction-
; level CHECK lines). The point is "no longer errors out at legalize
; time".

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: umul8_sat:
; CHECK: rts
define i8 @umul8_sat(i8 %a, i8 %b) {
  %r = call { i8, i1 } @llvm.umul.with.overflow.i8(i8 %a, i8 %b)
  %v = extractvalue { i8, i1 } %r, 0
  %o = extractvalue { i8, i1 } %r, 1
  %s = select i1 %o, i8 -1, i8 %v
  ret i8 %s
}

; CHECK-LABEL: umul16_sat:
; CHECK: rts
define i16 @umul16_sat(i16 %a, i16 %b) {
  %r = call { i16, i1 } @llvm.umul.with.overflow.i16(i16 %a, i16 %b)
  %v = extractvalue { i16, i1 } %r, 0
  %o = extractvalue { i16, i1 } %r, 1
  %s = select i1 %o, i16 -1, i16 %v
  ret i16 %s
}

; CHECK-LABEL: umul32_sat:
; CHECK: rts
define i32 @umul32_sat(i32 %a, i32 %b) {
  %r = call { i32, i1 } @llvm.umul.with.overflow.i32(i32 %a, i32 %b)
  %v = extractvalue { i32, i1 } %r, 0
  %o = extractvalue { i32, i1 } %r, 1
  %s = select i1 %o, i32 -1, i32 %v
  ret i32 %s
}

; s64 deliberately omitted — see comment header above and the long-long
; roadmap memory.

declare { i8,  i1 } @llvm.umul.with.overflow.i8(i8,  i8)
declare { i16, i1 } @llvm.umul.with.overflow.i16(i16, i16)
declare { i32, i1 } @llvm.umul.with.overflow.i32(i32, i32)
