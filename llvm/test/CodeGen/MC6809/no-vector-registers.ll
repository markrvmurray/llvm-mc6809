; RUN: opt -mtriple=mc6809 -slp-threshold=-100 -passes=slp-vectorizer -S %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 %s -o /dev/null

; The target has no vector registers, so the vectorizers must stay away however
; attractive the cost model finds a sequence. This shape became a <2 x i16> udiv
; and an extractelement the legalizer rejects. The threshold makes the check
; independent of the cost model.

; CHECK-NOT: x i16>
; CHECK-NOT: x i8>

define void @digits(i16 %v, ptr %p) {
  %t0 = udiv i16 %v, 1000
  %b0 = trunc i16 %t0 to i8
  %r0 = urem i8 %b0, 10
  store i8 %r0, ptr %p, align 1
  %p1 = getelementptr inbounds i8, ptr %p, i16 1
  %t1 = udiv i16 %v, 100
  %r1 = urem i16 %t1, 10
  %b1 = trunc i16 %r1 to i8
  store i8 %b1, ptr %p1, align 1
  %p2 = getelementptr inbounds i8, ptr %p, i16 2
  %t2 = udiv i16 %v, 10
  %r2 = urem i16 %t2, 10
  %b2 = trunc i16 %r2 to i8
  store i8 %b2, ptr %p2, align 1
  %p3 = getelementptr inbounds i8, ptr %p, i16 3
  %r3 = urem i16 %v, 10
  %b3 = trunc i16 %r3 to i8
  store i8 %b3, ptr %p3, align 1
  ret void
}
