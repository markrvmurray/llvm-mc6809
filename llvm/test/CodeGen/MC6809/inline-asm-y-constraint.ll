; RUN: llc -global-isel -mtriple=mc6809 -O2 %s -o - | FileCheck %s

; The 'y' inline-asm constraint binds a value to the Y index register.  It was
; paired with the IXc register class (X only) instead of IYc (Y only), so any
; use crashed InlineAsmLowering's getRegistersForValue ("AssignedReg should be
; a member of provided RC") — Y is not a member of the X class.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: roundtrip:
; CHECK:      tfr x,y
; CHECK:      leay 1,y
; CHECK:      tfr y,x
define i16 @roundtrip(i16 %n) {
  %r = call i16 asm sideeffect "leay 1,y", "=y,y"(i16 %n)
  ret i16 %r
}
