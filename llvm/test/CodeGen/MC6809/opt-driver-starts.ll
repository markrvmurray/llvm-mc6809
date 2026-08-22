; RUN: opt -S -passes=no-op-module %s | FileCheck %s

; opt registers every legacy pass name as a command-line option, so a target
; cl::opt spelled the same as one of the target's pass names aborts opt at
; startup with "registered more than once" -- before any test can run.

; CHECK: define void @f()
define void @f() {
  ret void
}
