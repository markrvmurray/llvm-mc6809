; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s

; A value moved between memory and a direct-page register goes through an
; accumulator, and when D holds something live the move used to be wrapped
; in `pshs d ... puls d`. A free X or Y now carries it instead, with nothing
; to save. A byte moved between a direct-page register and A or B goes
; straight there, not through the other half of D.

target triple = "mc6809-unknown-unknown"

; The 64-bit operands of a comparison's libcall are staged through
; direct-page registers while D holds another word.
; CHECK-LABEL: lt64:
; CHECK-NOT:   pshs d
; CHECK:       ldx {{[0-9]+}},s
; CHECK-NEXT:  stx <__rs0
; CHECK-NOT:   pshs d
; CHECK:       lbsr __cmpdi2
define i1 @lt64(i64 %a, i64 %b) {
  %c = icmp slt i64 %a, %b
  ret i1 %c
}
