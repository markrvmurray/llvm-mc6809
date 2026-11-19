; RUN: llc -mtriple=mc6809 -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -O0 %s -o - | FileCheck %s

; C23's minimum/maximum operations become calls of picolibc's functions:
; llvm.minimumnum and llvm.maximumnum (fminimum_num, fmaximum_num) through
; the generic libcall rule, llvm.minimum and llvm.maximum (fminimum,
; fmaximum) through a custom one, as the generic path has no entry for them.

; CHECK-LABEL: minimumnum_f32:
; CHECK: lbsr fminimum_numf{{$}}
define float @minimumnum_f32(float %a, float %b) {
  %r = call float @llvm.minimumnum.f32(float %a, float %b)
  ret float %r
}

; CHECK-LABEL: minimumnum_f64:
; CHECK: lbsr fminimum_num{{$}}
define double @minimumnum_f64(double %a, double %b) {
  %r = call double @llvm.minimumnum.f64(double %a, double %b)
  ret double %r
}

; CHECK-LABEL: maximumnum_f32:
; CHECK: lbsr fmaximum_numf{{$}}
define float @maximumnum_f32(float %a, float %b) {
  %r = call float @llvm.maximumnum.f32(float %a, float %b)
  ret float %r
}

; CHECK-LABEL: maximumnum_f64:
; CHECK: lbsr fmaximum_num{{$}}
define double @maximumnum_f64(double %a, double %b) {
  %r = call double @llvm.maximumnum.f64(double %a, double %b)
  ret double %r
}

; CHECK-LABEL: minimum_f32:
; CHECK: lbsr fminimumf{{$}}
define float @minimum_f32(float %a, float %b) {
  %r = call float @llvm.minimum.f32(float %a, float %b)
  ret float %r
}

; CHECK-LABEL: minimum_f64:
; CHECK: lbsr fminimum{{$}}
define double @minimum_f64(double %a, double %b) {
  %r = call double @llvm.minimum.f64(double %a, double %b)
  ret double %r
}

; CHECK-LABEL: maximum_f32:
; CHECK: lbsr fmaximumf{{$}}
define float @maximum_f32(float %a, float %b) {
  %r = call float @llvm.maximum.f32(float %a, float %b)
  ret float %r
}

; CHECK-LABEL: maximum_f64:
; CHECK: lbsr fmaximum{{$}}
define double @maximum_f64(double %a, double %b) {
  %r = call double @llvm.maximum.f64(double %a, double %b)
  ret double %r
}

