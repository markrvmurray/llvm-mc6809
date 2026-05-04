; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -stop-before=greedy %s -o - | FileCheck %s

; Regression for bug #209.
;
; Loop Strength Reduction transforms a loop's induction-variable use
; into the pair
;     %a = sub i16 0, %iv
;     %b = add i16 %base, %iv
; with the comparison `icmp eq %a, %neg_bound` doing an equivalent
; downward count. The MC6809 backend's byte-level decomposition of
; `%a = sub 0, %iv` mishandles the operand-pairing when `%iv` and
; `%base + %iv` end up in adjacent stack slots — the low byte is
; pulled from `%base + %iv` instead of `%iv`, so `%a` is computed as
; -(%base + %iv) instead of -%iv.  In picolibc's `getopt_internal`
; under aggressive LTO inlining (Os-lto + Os-hd6309-mame), this made
; the PERMUTE-ordering loop exit on its first iteration with the
; wrong "no more options" answer, then the function returned EOF in
; place of the expected '?' for an unknown short option.
;
; The fix disables LSR for MC6809 entirely (see MC6809PassConfig::
; addIRPasses).  This sentinel pins that decision: it lowers a
; canonical PERMUTE-style loop to MIR and checks that LSR did NOT
; rewrite the bound expression.  The original `sub i16 %smax, %a`
; must survive to the pre-greedy MIR exactly as written.  If LSR
; ever gets re-enabled, this test will fail (the bound becomes
; `sub i16 %a, %smax` and `%neg_iv = sub 0, %iv` is introduced).

declare i16 @llvm.smax.i16(i16, i16)
declare void @sink(ptr)

define i16 @bug209_permute_loop(i16 %a, i16 %argc, ptr %argv) {
entry:
  %smax = tail call i16 @llvm.smax.i16(i16 %a, i16 %argc)
  %bound = sub i16 %smax, %a
  br label %loop

loop:
  %iv = phi i16 [ 0, %entry ], [ %iv.next, %body ]
  %idx = phi i16 [ %a, %entry ], [ %idx.next, %body ]
  %done = icmp eq i16 %iv, %bound
  br i1 %done, label %exit, label %body

body:
  %p.addr = getelementptr inbounds [2 x i8], ptr %argv, i16 %idx
  %p = load ptr, ptr %p.addr, align 1
  call void @sink(ptr %p)
  %iv.next = add nuw nsw i16 %iv, 1
  %idx.next = add nuw nsw i16 %idx, 1
  br label %loop

exit:
  ret i16 %iv
}

; The pre-greedy MIR retains the original IR in its `--- |` header.
; The bound expression must remain `sub i16 %smax, %a` (smax minus a),
; NOT the LSR-rewritten `sub i16 %a, %smax` that exposes the
; backend miscompile.  Equally, no `sub i16 0, %iv` (LSR's "negated
; induction variable" pattern) may appear.
; CHECK:       %bound = sub i16 %smax, %a
; CHECK-NOT:   sub i16 %a, %smax
; CHECK-NOT:   sub i16 0, %iv
