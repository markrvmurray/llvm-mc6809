; NOTE: Hand-authored — pins folding a frame-index address into a pointer
; load/store at selection.
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs -filetype=asm < %s | FileCheck %s
;
; A pointer-typed (p0) load/store whose address is a frame index folds the
; frame index straight into the Load/Store_iPtr_Mem (as the scalar load patterns
; already do), instead of materialising the frame address with a separate
; LEA_Ptr_Imm into a scarce index register and dereferencing it.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; The third (stack-passed) pointer argument is returned: a single folded indexed
; load from the stack, no LEA to materialise the slot address first.
; CHECK-LABEL: third:
; CHECK-NOT: lea
; CHECK: ldx {{[0-9]+}},s
; CHECK-NEXT: rts
define ptr @third(ptr %a, ptr %b, ptr %c) {
entry:
  ret ptr %c
}

; Stack-passed pointer arguments are loaded directly from their frame slots.
; CHECK-LABEL: store3:
; CHECK: ldx {{[0-9]+}},u
; CHECK: ldy {{[0-9]+}},u
define void @store3(ptr %a, ptr %b, ptr %pp) {
entry:
  store ptr %b, ptr %pp, align 1
  ret void
}
