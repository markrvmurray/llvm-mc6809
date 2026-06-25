; NOTE: Hand-authored — pins folding an invariant function-pointer load into the
; indirect call that uses it (jsr [n,r]).
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs -filetype=asm < %s | FileCheck %s
;
; A function pointer passed as a stack argument and called repeatedly: an
; indirect call clobbers the index registers, so without this fold the pointer
; is reloaded into an index register before every call. MC6809 can read the
; pointer from its (invariant) stack slot as part of the call, so each call
; becomes a single `jsr [n,u]` with no register load and no index register tied
; up across the call.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: apply:
; CHECK: jsr [{{[0-9]+}},u]
; CHECK: jsr [{{[0-9]+}},u]
; The pointer is never reloaded into a register for a register-indirect call.
; CHECK-NOT: jsr ,
define void @apply(i16 %x, i16 %y, ptr %f, i16 %a, i16 %b) {
entry:
  call void %f(i16 %a)
  call void %f(i16 %b)
  ret void
}
