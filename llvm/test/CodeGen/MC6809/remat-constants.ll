; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s

; A constant live across a call is loaded again after it, not spilled to the
; frame and reloaded: `ldd #2` is as cheap as `ldd n,s` and needs no slot.

target triple = "mc6809-unknown-unknown"

declare void @h(i16, i16, i16)

; CHECK-LABEL: three:
; CHECK-NOT:   Folded Spill
; CHECK:       lbsr h
; CHECK-NOT:   Folded Reload
; CHECK:       ldd #2
; CHECK:       ldd #3
; CHECK:       lbsr h
define void @three() {
  call void @h(i16 1, i16 2, i16 3)
  call void @h(i16 1, i16 2, i16 3)
  ret void
}
