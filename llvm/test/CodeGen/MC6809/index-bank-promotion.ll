; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O2 -mc6809-enable-index-promotion=0 %s -o - | FileCheck %s --check-prefix=OFF

; A 16-bit value that is only loaded, stored, compared, stepped by a
; constant and moved has an index-register form for everything it does; it
; is re-classed into the index bank instead of contending with every other
; 16-bit value for the one accumulator (and losing, into an imaginary
; register with a pshs/puls pair around each access).

; memset's counter steps in Y (leay -1,y), the byte stays in B, the
; pointer walks in X.
define void @fill(ptr %p, i8 %c, i16 %n) {
entry:
  br label %loop
loop:
  %i = phi i16 [ %n, %entry ], [ %i1, %body ]
  %q = phi ptr [ %p, %entry ], [ %q1, %body ]
  %z = icmp eq i16 %i, 0
  br i1 %z, label %done, label %body
body:
  %i1 = add i16 %i, -1
  store i8 %c, ptr %q, align 1
  %q1 = getelementptr inbounds i8, ptr %q, i16 1
  br label %loop
done:
  ret void
}
; CHECK-LABEL: fill:
; CHECK-NOT:   <__rs
; CHECK:       lea{{[xy]}} -1,{{[xy]}}
; CHECK-NEXT:  stb ,{{[xy]}}+
; CHECK-NOT:   <__rs
; OFF-LABEL:   fill:
; OFF:         <__rs

; A constant returned in X needs no accumulator at all.
define i16 @k() {
  ret i16 1234
}
; CHECK-LABEL: k:
; CHECK:       ldx #1234
; CHECK-NEXT:  rts

; Three 16-bit values live at once do not fit two index registers: the
; candidates stay in the accumulator bank (no static-stack spill traffic).
define i16 @toupper16(i16 %c) {
entry:
  %0 = add i16 %c, -97
  %lt = icmp ult i16 %0, 26
  %sub = add i16 %c, -32
  %r = select i1 %lt, i16 %sub, i16 %c
  ret i16 %r
}
; CHECK-LABEL: toupper16:
; CHECK-NOT:   sstk
; CHECK:       cmpd #26
; CHECK-NOT:   sstk
