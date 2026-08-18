; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O1 -verify-machineinstrs %s -o - | FileCheck %s

; The byte a `*p++` loads is compared with two constants; the compares were
; selected first (bottom-up) and had already narrowed the value's class to
; what CMPA/CMPB accept when the load was fused into `ldb ,x+`. The fusion
; must constrain the value's class, not reset it (the verifier caught the
; reset at -Og in getsubopt).
define i16 @skip(ptr %p) {
entry:
  br label %loop
loop:
  %q = phi ptr [ %p, %entry ], [ %q1, %loop ]
  %c = load i8, ptr %q, align 1
  %q1 = getelementptr inbounds i8, ptr %q, i16 1
  %sp = icmp eq i8 %c, 32
  %tb = icmp eq i8 %c, 9
  %ws = or i1 %sp, %tb
  br i1 %ws, label %loop, label %done
done:
  %r = ptrtoint ptr %q to i16
  ret i16 %r
}
; CHECK-LABEL: skip:
; CHECK:       ld{{[ab]}} ,{{[xy]}}+
