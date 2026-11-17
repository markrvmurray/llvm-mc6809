; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,M6809
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,H6309

; A constant 16-bit shift by a whole byte or more moves the byte that
; survives and shifts that one byte by what is left, instead of shifting the
; whole value one bit at a time. An arithmetic shift fills with the sign,
; which is SEX; a sign byte shifted right logically is an AND. Signed
; division by a power of two, built from those, comes out short.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: shl8:
; CHECK-NOT:   asl
; CHECK-NOT:   rol
; CHECK:       rts
define i16 @shl8(i16 %x) {
  %r = shl i16 %x, 8
  ret i16 %r
}

; Only the remaining two bits are shifted, in one byte.
; CHECK-LABEL: lshr10:
; CHECK:       lsr{{[ab]}}
; CHECK-NEXT:  lsr{{[ab]}}
; CHECK-NOT:   lsr
; CHECK-NOT:   ror
; CHECK:       rts
define i16 @lshr10(i16 %x) {
  %r = lshr i16 %x, 10
  ret i16 %r
}

; CHECK-LABEL: ashr15:
; CHECK:       sex
; CHECK-NOT:   asr
; CHECK:       rts
define i16 @ashr15(i16 %x) {
  %r = ashr i16 %x, 15
  ret i16 %r
}

; The rounding bias is the sign masked to 3, then two arithmetic shifts.
; CHECK-LABEL: sdiv4:
; CHECK:       sex
; CHECK:       andb #3
; M6809:       asra
; M6809-NEXT:  rorb
; M6809-NEXT:  asra
; M6809-NEXT:  rorb
; H6309:       asrd
; H6309-NEXT:  asrd
; CHECK:       rts
define i16 @sdiv4(i16 %x) {
  %r = sdiv i16 %x, 4
  ret i16 %r
}
