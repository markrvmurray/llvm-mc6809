; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; A byte compare whose loaded operand is on the LHS of an equality
; (`if (*p == x)`) must fold the load into the compare's addressing mode
; (`cmpb ,x`) -- even when the enclosing function contains a call, and for the
; select-lowered value forms `(*p == x) ? -1 : 0` / `-(*p == x)`. The
; load-on-LHS path (predicate-swap in the G_BRCOND / G_ICMP selector arm) once
; dropped the fold under those conditions and emitted a clumsy reg-vs-reg push
; form (`lda ,x; pshs b; cmpa ,s+`). Guard against that regression.

declare void @e()

; The core case: load on the LHS of the compare, with a call in the function.
define void @load_lhs_eq_with_call(ptr readonly %p, i8 signext %x) {
; CHECK-LABEL: load_lhs_eq_with_call:
; CHECK:         cmpb ,x
; CHECK-NOT:     pshs b
; CHECK-NOT:     cmpa ,s
entry:
  %v = load i8, ptr %p, align 1
  %cmp = icmp eq i8 %v, %x
  br i1 %cmp, label %then, label %end

then:
  tail call void @e()
  br label %end

end:
  ret void
}

; Select-lowered value form: `(*p == x) ? -1 : 0` (sext of the setcc) must also
; fold the LHS load rather than pushing.
define i16 @load_lhs_select_m1(ptr readonly %p, i8 signext %x) {
; CHECK-LABEL: load_lhs_select_m1:
; CHECK:         cmpb ,x
; CHECK-NOT:     pshs b
; CHECK-NOT:     cmpa ,s
entry:
  %v = load i8, ptr %p, align 1
  %cmp = icmp eq i8 %v, %x
  %cond = sext i1 %cmp to i16
  ret i16 %cond
}
