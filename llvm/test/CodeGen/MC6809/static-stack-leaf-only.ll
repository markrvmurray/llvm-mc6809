; RUN: llc -mtriple=mc6809 -O2 -mc6809-static-stack-dp-avail=0 %s -o - | FileCheck %s

; A static frame is taken only where it frees the frame pointer. Both
; functions below are provably non-reentrant (internal, only called from one
; place, no external-call cycle). The leaf gets a static frame and no frame
; pointer at all; the function that calls it keeps its dynamic U-relative
; frame -- a static frame would cost it a byte and a cycle per access and buy
; nothing, since it needs U as a frame pointer across the call anyway.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

; CHECK-LABEL: leaf:
; CHECK-NOT:     pshs{{.*}}u
; CHECK-NOT:     ,u
; CHECK:         .Lleaf_sstk
; CHECK:         rts
define internal i16 @leaf(ptr %p, i16 %n) noinline norecurse {
entry:
  %a = alloca [8 x i16]
  br label %loop
loop:
  %i = phi i16 [ 0, %entry ], [ %i.next, %loop ]
  %s = phi i16 [ 0, %entry ], [ %s.next, %loop ]
  %m = and i16 %i, 7
  %slot = getelementptr [8 x i16], ptr %a, i16 0, i16 %m
  %src = getelementptr i16, ptr %p, i16 %i
  %v = load i16, ptr %src
  store volatile i16 %v, ptr %slot
  %w = load volatile i16, ptr %slot
  %s.next = add i16 %s, %w
  %i.next = add i16 %i, 1
  %done = icmp eq i16 %i.next, %n
  br i1 %done, label %exit, label %loop
exit:
  ret i16 %s.next
}

; CHECK-LABEL: caller:
; CHECK:         pshs{{.*}}u
; CHECK:         tfr s,u
; CHECK-NOT:     _sstk
; CHECK:         ,u
; CHECK-NOT:     _sstk
; CHECK:         rts
define internal i16 @caller(ptr %p, i16 %n) noinline norecurse {
entry:
  %a = alloca [8 x i16]
  br label %loop
loop:
  %i = phi i16 [ 0, %entry ], [ %i.next, %loop ]
  %s = phi i16 [ 0, %entry ], [ %s.next, %loop ]
  %m = and i16 %i, 7
  %slot = getelementptr [8 x i16], ptr %a, i16 0, i16 %m
  %src = getelementptr i16, ptr %p, i16 %i
  %v = load i16, ptr %src
  store volatile i16 %v, ptr %slot
  %w = load volatile i16, ptr %slot
  %r = call i16 @leaf(ptr %p, i16 %w)
  %s.next = add i16 %s, %r
  %i.next = add i16 %i, 1
  %done = icmp eq i16 %i.next, %n
  br i1 %done, label %exit, label %loop
exit:
  ret i16 %s.next
}

define i16 @api(ptr %p, i16 %n) norecurse {
  %r = call i16 @caller(ptr %p, i16 %n)
  ret i16 %r
}
