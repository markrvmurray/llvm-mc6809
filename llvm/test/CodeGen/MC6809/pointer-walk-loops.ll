; RUN: llc -mtriple=mc6809 -mattr=-static-stack -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -mattr=-static-stack -mcpu=hd6309 -O2 %s -o - | FileCheck %s

; Loops that walk a pointer through memory keep that one pointer in an index
; register with the step folded into the access (`ldb ,x+`): the loop
; strength reducer's addressing-mode model no longer prefers a base plus a
; counter (`ldb d,x` ties up the accumulator), a value the loop wants after
; the loop is read back through the stepped pointer, and a step that sits
; after the exit test is brought up to the access.

; strlen: the pointer before the step is wanted after the loop (`p - start`).
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
; CHECK:       ld{{[ab]}} ,{{[xy]}}+
; CHECK-NEXT:  bne
; CHECK:       lea{{[xy]}} -1,{{[xy]}}
; CHECK-NOT:   <__rs
; CHECK:       rts

; strcpy: two walking pointers, byte through B; the store sets Z, no tst.
define void @cpy(ptr %d, ptr %s) {
entry:
  br label %loop
loop:
  %pd = phi ptr [ %d, %entry ], [ %pd1, %loop ]
  %ps = phi ptr [ %s, %entry ], [ %ps1, %loop ]
  %c = load i8, ptr %ps, align 1
  store i8 %c, ptr %pd, align 1
  %pd1 = getelementptr inbounds i8, ptr %pd, i16 1
  %ps1 = getelementptr inbounds i8, ptr %ps, i16 1
  %z = icmp eq i8 %c, 0
  br i1 %z, label %done, label %loop
done:
  ret void
}
; CHECK-LABEL: cpy:
; CHECK:       ld{{[ab]}} ,{{[xy]}}+
; CHECK-NEXT:  st{{[ab]}} ,{{[xy]}}+
; CHECK-NEXT:  bne

; memcmp-like: the steps live past the exit test in the latch; the first
; is hoisted to its load and folded, the second load feeds the compare
; straight from memory.
define i8 @cmpn(ptr %a, ptr %b, i16 %n) {
entry:
  br label %loop
loop:
  %pa = phi ptr [ %a, %entry ], [ %pa1, %latch ]
  %pb = phi ptr [ %b, %entry ], [ %pb1, %latch ]
  %i = phi i16 [ %n, %entry ], [ %i1, %latch ]
  %ca = load i8, ptr %pa, align 1
  %cb = load i8, ptr %pb, align 1
  %ne = icmp ne i8 %ca, %cb
  br i1 %ne, label %done, label %latch
latch:
  %pa1 = getelementptr inbounds i8, ptr %pa, i16 1
  %pb1 = getelementptr inbounds i8, ptr %pb, i16 1
  %i1 = add i16 %i, -1
  %z = icmp eq i16 %i1, 0
  br i1 %z, label %done, label %loop
done:
  %r = phi i8 [ %ca, %loop ], [ 0, %latch ]
  ret i8 %r
}
; CHECK-LABEL: cmpn:
; CHECK:       ld{{[ab]}} ,{{[xy]}}+
; CHECK-NEXT:  cmp{{[ab]}} ,{{[xy]}}
; CHECK-NEXT:  bne
