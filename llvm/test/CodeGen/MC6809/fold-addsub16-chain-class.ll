; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O1 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s

; An add whose result feeds a subtract straight from its two bytes: the
; 16-bit add fold gives the pair the shape of a whole value (merged, then
; split again) so that both fold. The split bytes must be in A or B, the
; class the split instructions define. They were given the class of the
; add's result, which for an add of a constant is the wider byte class, and
; the machine verifier rejected the split. Reduced from picolibc's fma at
; -Og.

target triple = "mc6809-unknown-unknown"

; The add folds; the subtract stays in bytes, since the comparison reads its
; flags.
; CHECK-LABEL: add_then_sub:
; CHECK:       addd #1
; CHECK:       subb
; CHECK-NEXT:  sbca
define i16 @add_then_sub(i16 %x, i16 %y) {
  %a = add i16 %x, 1
  %s = sub i16 %a, %y
  %neg = icmp slt i16 %s, 0
  %m = sub i16 0, %s
  %r = select i1 %neg, i16 0, i16 %m
  ret i16 %r
}
