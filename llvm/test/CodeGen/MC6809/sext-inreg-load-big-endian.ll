; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s

; A sign extension of the low bits of a loaded value must not become a
; narrower sign-extending load from the same address: the 6809 is
; big-endian, so the low bytes of a 32-bit value are the last two (or the
; last one), not the first. The 6309 has a legal 32-bit sign-extending
; load, so it is where the generic combine fired: it read the argument's
; high word at 2,s where the low word is at 4,s (and its top byte for the
; low byte at 5,s).

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: sext16in32:
; CHECK-NOT:   {{ldd|ldw}} 2,s
; CHECK:       rts
define i32 @sext16in32(i32 %a) {
  %s = shl i32 %a, 16
  %r = ashr exact i32 %s, 16
  ret i32 %r
}

; CHECK-LABEL: sext8in32:
; CHECK-NOT:   ldb {{[234]}},s
; CHECK:       rts
define i32 @sext8in32(i32 %a) {
  %s = shl i32 %a, 24
  %r = ashr exact i32 %s, 24
  ret i32 %r
}
