; RUN: llc -mtriple=mc6809 -O1 -verify-machineinstrs -print-after=instruction-select < %s -o /dev/null 2>&1 | FileCheck %s
; RUN: llc -mtriple=mc6809 -O1 -verify-machineinstrs < %s -o /dev/null

; A compare re-fused with its branch into CompareBranch_i8_Imm keeps the
; compare's source register, but the fused form takes a narrower class
; (A, B, E or F: ACC8_CMP) than the plain compare (ACC8, which also holds
; direct-page bytes). The source must be constrained to it, or the verifier
; rejects the function: picolibc's fdim, fmin, fmax and hypot at -Og with
; floating point, where the byte is an `fcmp` library call's result.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: Machine code for function pick
; CHECK:       CompareBranch_i8_Imm {{[0-9]+}}, {{%[0-9]+}}:acc8_cmp, 0
define double @pick(double %x, double %y) {
entry:
  %cmp = fcmp ogt double %x, %y
  %sub = fsub double %x, %y
  %cond = select i1 %cmp, double %sub, double 0.000000e+00
  ret double %cond
}
