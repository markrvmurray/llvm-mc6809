; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc -verify-machineinstrs -O0 --filetype=asm < %s | FileCheck %s -check-prefixes=CHECK
; RUN: llc -verify-machineinstrs -O0 --filetype=asm -mcpu hd6309 < %s | FileCheck %s -check-prefixes=CHECK-HD6309
target triple = "mc6809"

define i8 @foo() #0 {
; CHECK-LABEL: foo:
; CHECK:       ; %bb.0: ; %entry
; CHECK-NEXT:    ldb #85
; CHECK-NEXT:    rts
;
; CHECK-HD6309-LABEL: foo:
; CHECK-HD6309:       ; %bb.0: ; %entry
; CHECK-HD6309-NEXT:    ldb #85
; CHECK-HD6309-NEXT:    rts
entry:
  ret i8 85
}
