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

; CHECK-LABEL: touch:
; CHECK: leax mc6809_os9_data(initialized_value),u
; CHECK: leax mc6809_os9_data(uninitialized_value),u
; CHECK: leax mc6809_os9_data(external_value),u
; CHECK: leax .Lmessage,pc
; CHECK-NOT: leax initialized_value,pc
; CHECK-NOT: leax uninitialized_value,pc
; CHECK-NOT: tfr s,u
