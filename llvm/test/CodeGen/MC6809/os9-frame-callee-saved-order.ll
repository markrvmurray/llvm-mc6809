; RUN: llc -mtriple=mc6809-unknown-os9 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: without a frame pointer the outgoing-call arguments are
; written at the bottom of the frame (`,s`, `2,s` ...), so the frame must be
; allocated BELOW the callee-saved registers -- push first, then LEAS -- or
; the first outgoing argument lands on top of a saved register.  An OS-9
; program never has a frame pointer (U is the process data-area base), so
; every calling function takes this path.
;
; The incoming arguments then sit beyond the saved registers and the return
; address: frame (10) + saved Y (2) + return PC (2) = 14.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-os9"

declare i16 @g(i16, i16, i16, i16)

define i16 @caller(i16 %a, i16 %b, ptr %p) {
; CHECK-LABEL: caller:
; CHECK:         pshs y
; CHECK-NEXT:    leas -10,s
; CHECK:         ldd 14,s
; CHECK:         lbsr g
; CHECK:         leas 10,s
; CHECK-NEXT:    puls y
; CHECK-NEXT:    rts
  %v = load i16, ptr %p
  %r = call i16 @g(i16 %a, i16 %b, i16 %v, i16 %b)
  %s = call i16 @g(i16 %r, i16 %v, i16 %a, i16 %b)
  %t = add i16 %r, %s
  ret i16 %t
}
