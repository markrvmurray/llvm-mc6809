; RUN: llc -mtriple=mc6809 -O0 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs %s -o /dev/null

; InstCombine fuses a sin/cos pair of the same argument into llvm.sincos,
; which reaches the backend as G_FSINCOS. It is a libcall here, like the
; rest of the transcendentals — but sincos/sincosf are gated on a GNU
; environment in the generic libcall table, and this target is not one,
; so they have to be declared available by the MC6809 system library
; (picolibc provides them). Without that, every libm function computing
; both sin and cos of one argument — csin, ccos, ctanh and friends —
; failed to legalize under fast-math.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

declare { float, float } @llvm.sincos.f32(float)
declare { double, double } @llvm.sincos.f64(double)

define float @sincos_f32(float %x) {
; CHECK-LABEL: sincos_f32:
; CHECK:         sincosf
  %r = call { float, float } @llvm.sincos.f32(float %x)
  %s = extractvalue { float, float } %r, 0
  %c = extractvalue { float, float } %r, 1
  %sum = fadd float %s, %c
  ret float %sum
}

define double @sincos_f64(double %x) {
; CHECK-LABEL: sincos_f64:
; CHECK:         sincos
  %r = call { double, double } @llvm.sincos.f64(double %x)
  %s = extractvalue { double, double } %r, 0
  %c = extractvalue { double, double } %r, 1
  %sum = fadd double %s, %c
  ret double %sum
}
