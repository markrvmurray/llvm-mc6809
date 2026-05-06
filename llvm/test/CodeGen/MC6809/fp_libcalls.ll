; Bug #162 sentinel: MC6839 soft-float libcall emission.
; Verifies that FP operations lower to the expected soft-float libcall names
; and that the calls appear in the output (X = result ptr, args on stack).
;
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 %s -o - \
; RUN:   | FileCheck %s

; ---- f32 arithmetic ---------------------------------------------------------

define float @test_fadd(float %a, float %b) {
; CHECK-LABEL: test_fadd:
; CHECK: lbsr __addsf3
  %r = fadd float %a, %b
  ret float %r
}

define float @test_fsub(float %a, float %b) {
; CHECK-LABEL: test_fsub:
; CHECK: lbsr __subsf3
  %r = fsub float %a, %b
  ret float %r
}

define float @test_fmul(float %a, float %b) {
; CHECK-LABEL: test_fmul:
; CHECK: lbsr __mulsf3
  %r = fmul float %a, %b
  ret float %r
}

define float @test_fdiv(float %a, float %b) {
; CHECK-LABEL: test_fdiv:
; CHECK: lbsr __divsf3
  %r = fdiv float %a, %b
  ret float %r
}

; ---- f64 arithmetic ---------------------------------------------------------

define double @test_fadd_f64(double %a, double %b) {
; CHECK-LABEL: test_fadd_f64:
; CHECK: lbsr __adddf3
  %r = fadd double %a, %b
  ret double %r
}

define double @test_fdiv_f64(double %a, double %b) {
; CHECK-LABEL: test_fdiv_f64:
; CHECK: lbsr __divdf3
  %r = fdiv double %a, %b
  ret double %r
}

; ---- sqrt -------------------------------------------------------------------

define float @test_sqrt(float %a) {
; CHECK-LABEL: test_sqrt:
; CHECK: lbsr __sqrtsf2
  %r = call float @llvm.sqrt.f32(float %a)
  ret float %r
}

declare float @llvm.sqrt.f32(float)

; ---- comparisons ------------------------------------------------------------

define i1 @test_fcmp_olt(float %a, float %b) {
; CHECK-LABEL: test_fcmp_olt:
; CHECK: lbsr __ltsf2
  %r = fcmp olt float %a, %b
  ret i1 %r
}

define i1 @test_fcmp_oeq(float %a, float %b) {
; CHECK-LABEL: test_fcmp_oeq:
; CHECK: lbsr __eqsf2
  %r = fcmp oeq float %a, %b
  ret i1 %r
}

define i1 @test_fcmp_olt_f64(double %a, double %b) {
; CHECK-LABEL: test_fcmp_olt_f64:
; CHECK: lbsr __ltdf2
  %r = fcmp olt double %a, %b
  ret i1 %r
}

; ---- type conversions -------------------------------------------------------

define float @test_sitofp(i32 %a) {
; CHECK-LABEL: test_sitofp:
; CHECK: lbsr __floatsisf
  %r = sitofp i32 %a to float
  ret float %r
}

define i32 @test_fptosi(float %a) {
; CHECK-LABEL: test_fptosi:
; CHECK: lbsr __fixsfsi
  %r = fptosi float %a to i32
  ret i32 %r
}

define double @test_fpext(float %a) {
; CHECK-LABEL: test_fpext:
; CHECK: lbsr __extendsfdf2
  %r = fpext float %a to double
  ret double %r
}

define float @test_fptrunc(double %a) {
; CHECK-LABEL: test_fptrunc:
; CHECK: lbsr __truncdfsf2
  %r = fptrunc double %a to float
  ret float %r
}
