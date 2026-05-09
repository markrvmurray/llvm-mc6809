; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O1 < %s | FileCheck %s
;
; Bug #241: legalizer must lower the full <math.h> trig / hyperbolic /
; inverse-trig / exp / mod family to libcalls. Without these rules the
; legalizer aborts at -Og / -Ofast where less inlining preserves the
; bare opcodes.
;
; This sentinel exercises every libcall that #241 added to the rule set.
; All must reach selection without "unable to legalize" — verified by the
; presence of the corresponding lbsr.
;
; CHECK-LABEL: f_atan:
; CHECK: lbsr	atan
; CHECK-LABEL: f_atan2:
; CHECK: lbsr	atan2
; CHECK-LABEL: f_sinh:
; CHECK: lbsr	sinh
; CHECK-LABEL: f_cosh:
; CHECK: lbsr	cosh
; CHECK-LABEL: f_tanh:
; CHECK: lbsr	tanh
; CHECK-LABEL: f_tan:
; CHECK: lbsr	tan
; CHECK-LABEL: f_asin:
; CHECK: lbsr	asin
; CHECK-LABEL: f_acos:
; CHECK: lbsr	acos
; CHECK-LABEL: f_modf:
; CHECK: lbsr	modf

declare double @llvm.atan.f64(double)
declare double @llvm.atan2.f64(double, double)
declare double @llvm.sinh.f64(double)
declare double @llvm.cosh.f64(double)
declare double @llvm.tanh.f64(double)
declare double @llvm.tan.f64(double)
declare double @llvm.asin.f64(double)
declare double @llvm.acos.f64(double)
declare {double, double} @llvm.modf.f64(double)

define double @f_atan(double %x) { %r = call double @llvm.atan.f64(double %x)
  ret double %r }
define double @f_atan2(double %x, double %y) { %r = call double @llvm.atan2.f64(double %x, double %y)
  ret double %r }
define double @f_sinh(double %x) { %r = call double @llvm.sinh.f64(double %x)
  ret double %r }
define double @f_cosh(double %x) { %r = call double @llvm.cosh.f64(double %x)
  ret double %r }
define double @f_tanh(double %x) { %r = call double @llvm.tanh.f64(double %x)
  ret double %r }
define double @f_tan(double %x)  { %r = call double @llvm.tan.f64(double %x)
  ret double %r }
define double @f_asin(double %x) { %r = call double @llvm.asin.f64(double %x)
  ret double %r }
define double @f_acos(double %x) { %r = call double @llvm.acos.f64(double %x)
  ret double %r }
define double @f_modf(double %x) { %r = call {double, double} @llvm.modf.f64(double %x)
  %v = extractvalue {double, double} %r, 0
  ret double %v }
