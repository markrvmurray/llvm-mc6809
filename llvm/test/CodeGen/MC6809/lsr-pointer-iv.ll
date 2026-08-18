; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s

; The loop strength reducer used to rewrite `while (*p) p++` as `p[i]`
; because the addressing-mode model, inherited from the 6502 port, called a
; register-plus-register address free: on the 6809 that is `ldb d,x`, which
; ties up the only 16-bit accumulator with the counter (and the counter then
; lived in an imaginary register). Register-plus-register is no longer
; legal to it, and a solution its own cost model rates worse than the
; original code is dropped, so the loop keeps its single pointer.
define i16 @len(ptr %s) {
entry:
  br label %loop
loop:
  %p = phi ptr [ %s, %entry ], [ %inc, %loop ]
  %c = load i8, ptr %p, align 1
  %z = icmp eq i8 %c, 0
  %inc = getelementptr inbounds i8, ptr %p, i16 1
  br i1 %z, label %done, label %loop
done:
  %a = ptrtoint ptr %p to i16
  %b = ptrtoint ptr %s to i16
  %d = sub i16 %a, %b
  ret i16 %d
}
; CHECK-LABEL: len:
; CHECK-NOT:   <__rs
; CHECK-NOT:   ld{{[ab]}} d,
; CHECK:       ld{{[ab]}} ,{{[xy]}}
; CHECK-NOT:   <__rs
; CHECK-NOT:   ld{{[ab]}} d,
; CHECK:       rts
