; RUN: llc -O2 -mtriple=mc6809 -mattr=+static-stack -verify-machineinstrs %s -o - | FileCheck %s
; RUN: llc -O2 -mtriple=mc6809 -mattr=+static-stack -mc6809-static-stack-dp-avail=0 -verify-machineinstrs %s -o - | FileCheck %s

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; A non-reentrant function whose i64 subtracts spill to the static frame drives
; the byte carry chain (SubSetCarry / SubSetCarryUse, linked by PHANTOM_CARRY
; scheduling operands) through the _Mem -> _Sym frame-index rewrite. That
; rewrite must preserve the dynamic PHANTOM_CARRY implicit operands, which no
; instruction descriptor supplies. If it drops them, the rewritten producer
; stops defining the phantom_carry its consumer still uses, and
; -verify-machineinstrs aborts with "Using an undefined physical register".
; The -verify-machineinstrs RUN lines are the regression guard; the -Og picolibc
; build (which passes -verify-machineinstrs) hit exactly this. Both extended and
; direct-page frame placement go through the same _Sym rewrite.

; CHECK-LABEL: ss_isub:
; The i64 subtract chain lowers to a sub root followed by sbc borrow-consumers
; against the static frame.
; CHECK: sub{{[ab]}}
; CHECK: sbc
define i64 @ss_isub(i64 %a, i64 %b, i64 %c, i64 %d, i64 %e) #0 {
  %s1 = sub i64 %a, %b
  %s2 = sub i64 %c, %d
  %s3 = sub i64 %s1, %e
  %s4 = sub i64 %s2, %s3
  %s5 = sub i64 %s4, %a
  ret i64 %s5
}

attributes #0 = { nounwind "nonreentrant" }
