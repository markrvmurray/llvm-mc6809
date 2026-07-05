; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809-unknown-os9 -relocation-model=pic -O0 %s -o - | FileCheck %s

target triple = "mc6809-unknown-os9"

@initialized_value = global i16 4660, align 2
@uninitialized_value = global i16 0, align 2
@external_value = external global i16, align 2
@message = private unnamed_addr constant [3 x i8] c"hi\00", align 1

define i16 @touch() {
entry:
  store i16 22136, ptr @initialized_value, align 2
  store i16 -25924, ptr @uninitialized_value, align 2
  store i16 1, ptr @external_value, align 2
  %a = load i16, ptr @initialized_value, align 2
  %b = load i16, ptr @uninitialized_value, align 2
  %c = load i16, ptr @external_value, align 2
  %p = ptrtoint ptr @message to i16
  %sum0 = add i16 %a, %b
  %sum1 = add i16 %sum0, %c
  %sum2 = add i16 %sum1, %p
  ret i16 %sum2
}

; Writable OS9 globals are addressed SU-relative via the deferred
; Lea_iPtr_OS9Sym pseudo: LEA{X,Y} mc6809_os9_data(g),u, with the index register
; the allocator's choice (not pinned to IX + a COPY). The read-only message
; stays PC-relative.
; CHECK-LABEL: touch:
; CHECK: lea{{[xyu]}} mc6809_os9_data(initialized_value),u
; CHECK: lea{{[xyu]}} mc6809_os9_data(uninitialized_value),u
; CHECK: lea{{[xyu]}} mc6809_os9_data(external_value),u
; CHECK: lea{{[xyu]}} .Lmessage,pc
; A writable global must never be PC-relative (it lives in the SU-relative data
; area), and the frame pointer is never rebuilt.
; CHECK-NOT: initialized_value,pc
; CHECK-NOT: uninitialized_value,pc
; CHECK-NOT: tfr s,u
