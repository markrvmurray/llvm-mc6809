; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
;
; G_VAARG legalization regression test for bug #85.
;
; clang lowers `va_arg(ap, long)` and `va_arg(ap, long long)` to
; G_VAARG with s32 / s64 destination types. Before bug #85's fix the
; MC6809 G_VAARG legalizer rule only accepted types in LegalTypes
; ({p, s8, s16} on basic 6809), so wider va_arg pickups fell to
; "unable to legalize". The fix extends the customForCartesianProduct
; list to {p, s8, s16, s32, s64}, and the existing legalizeVAArg
; custom handler is type-agnostic — it loads ValTy.getSizeInBytes()
; bytes and bumps the va_list pointer by that amount.
;
; This test only checks that the functions compile (no instruction-
; level CHECK lines) so it doesn't lock in specific codegen choices.
; The point is "no longer errors out at legalize time".

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: get_i8:
; CHECK: rts
define i8 @get_i8(ptr %ap) {
  %v = va_arg ptr %ap, i8
  ret i8 %v
}

; CHECK-LABEL: get_i16:
; CHECK: rts
define i16 @get_i16(ptr %ap) {
  %v = va_arg ptr %ap, i16
  ret i16 %v
}

; CHECK-LABEL: get_i32:
; CHECK: rts
define i32 @get_i32(ptr %ap) {
  %v = va_arg ptr %ap, i32
  ret i32 %v
}

; CHECK-LABEL: get_i64:
; CHECK: rts
define i64 @get_i64(ptr %ap) {
  %v = va_arg ptr %ap, i64
  ret i64 %v
}

; CHECK-LABEL: get_ptr:
; CHECK: rts
define ptr @get_ptr(ptr %ap) {
  %v = va_arg ptr %ap, ptr
  ret ptr %v
}
