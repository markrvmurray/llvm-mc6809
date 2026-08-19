; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: a shift amount of an odd width is neither the by-one form
; nor the by-a-byte one, and clamping it to that range leaves it between
; them with no rule to match -- the backend used to give up on it.  Such
; amounts turn up in lowered bit manipulation, not just in hand-written IR.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: shr3:
; CHECK: rts
define i8 @shr3(i8 %x, i3 %n) {
  %z = zext i3 %n to i8
  %r = lshr i8 %x, %z
  ret i8 %r
}

; CHECK-LABEL: shl5:
; CHECK: rts
define i16 @shl5(i16 %x, i5 %n) {
  %z = zext i5 %n to i16
  %r = shl i16 %x, %z
  ret i16 %r
}
