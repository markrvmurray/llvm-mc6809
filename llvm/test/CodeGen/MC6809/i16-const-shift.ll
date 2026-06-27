; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - \
; RUN:   | FileCheck %s --check-prefixes=CHECK,M6809
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - \
; RUN:   | FileCheck %s --check-prefixes=CHECK,HD6309
;
; Constant i16 shifts by 1..15 are inlined as the native single-bit carry chain
; on the value's own 16-bit home -- no __ashlhi3 / __lshrhi3 / __ashrhi3 libcall.
;   base 6809 uses ASLB+ROLA (shl), LSRA+RORB (lshr), ASRA+RORB (ashr).
;   hd6309 uses ASLD / LSRD / ASRD (one wide instruction per bit).

define i16 @shl4(i16 %x) {
; CHECK-LABEL: shl4:
; CHECK-NOT: __ashlhi3
; M6809: aslb
; M6809-NEXT: rola
; HD6309: asld
; HD6309-NEXT: asld
entry:
  %r = shl i16 %x, 4
  ret i16 %r
}

define i16 @lshr7(i16 %x) {
; CHECK-LABEL: lshr7:
; CHECK-NOT: __lshrhi3
; M6809: lsra
; M6809-NEXT: rorb
; HD6309: lsrd
entry:
  %r = lshr i16 %x, 7
  ret i16 %r
}

define i16 @ashr1(i16 %x) {
; CHECK-LABEL: ashr1:
; CHECK-NOT: __ashrhi3
; M6809: asra
; M6809-NEXT: rorb
; HD6309: asrd
entry:
  %r = ashr i16 %x, 1
  ret i16 %r
}

define i16 @shl15(i16 %x) {
; CHECK-LABEL: shl15:
; CHECK-NOT: __ashlhi3
entry:
  %r = shl i16 %x, 15
  ret i16 %r
}
