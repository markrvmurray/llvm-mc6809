; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s

; A value moved between memory and a direct-page register goes through an
; accumulator, and when D holds something live the move used to be wrapped
; in `pshs d ... puls d`. A free X or Y now carries it instead, with nothing
; to save. A byte moved between a direct-page register and A or B goes
; straight there, not through the other half of D.

target triple = "mc6809-unknown-unknown"

; Three 16-bit stack arguments are staged into direct-page registers for the
; comparison's libcall while D holds the fourth.
; CHECK-LABEL: lt32:
; CHECK-NOT:   pshs d
; CHECK:       ldx {{[0-9]+}},s
; CHECK-NEXT:  stx <__rs0
; CHECK-NOT:   pshs d
; CHECK:       lbsr __cmpsi2
define i1 @lt32(i32 %a, i32 %b) {
  %c = icmp slt i32 %a, %b
  ret i1 %c
}
