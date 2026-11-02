; RUN: llc -mtriple=mc6809 -verify-machineinstrs < %s | FileCheck %s

; A load through a pointer directly followed by the step of that pointer folds
; to an auto-increment form. The step is then the instruction the pass would
; visit next, so erasing it must not leave the walk on a deleted instruction.
; This used to crash the compiler.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: first_then_rest:
; CHECK: ldd ,x++
define i16 @first_then_rest(ptr %p, i16 %n) {
entry:
  %count = load i16, ptr %p, align 1
  %rest = getelementptr i8, ptr %p, i16 2
  br label %loop

loop:
  %cur = phi ptr [ %rest, %entry ], [ null, %body ]
  %more = icmp slt i16 %n, %count
  br i1 %more, label %body, label %done

body:
  %v = load i16, ptr %cur, align 1
  %zero = icmp eq i16 %v, 0
  br i1 %zero, label %done, label %loop

done:
  ret i16 0
}
