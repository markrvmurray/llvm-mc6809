; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 %s -o - | FileCheck %s

; Rounding to long long has no inline form, so it is the libm call.

declare i64 @llvm.llrint.i64.f64(double)
declare i64 @llvm.llrint.i64.f32(float)

; CHECK-LABEL: from_double:
; CHECK: llrint
define i64 @from_double(double %x) {
  %r = call i64 @llvm.llrint.i64.f64(double %x)
  ret i64 %r
}

; CHECK-LABEL: from_float:
; CHECK: llrintf
define i64 @from_float(float %x) {
  %r = call i64 @llvm.llrint.i64.f32(float %x)
  ret i64 %r
}
