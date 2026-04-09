; RUN: llc -global-isel -O0 < %s | FileCheck %s
; RUN: llc -global-isel -O1 < %s | FileCheck %s
; RUN: llc -global-isel -O2 < %s | FileCheck %s
; RUN: llc -global-isel -O3 < %s | FileCheck %s

target triple = "mc6809-unknown-unknown"

@g = external global i64

; Basic i64 add with carry propagation through 4 words.
; CHECK-LABEL: add64:
; CHECK: addd
; CHECK: adca
; CHECK: rts
define i64 @add64(i64 %a, i64 %b) {
  %r = add i64 %a, %b
  ret i64 %r
}

; i64 sub with borrow propagation.
; CHECK-LABEL: sub64:
; CHECK: subd
; CHECK: rts
define i64 @sub64(i64 %a, i64 %b) {
  %r = sub i64 %a, %b
  ret i64 %r
}

; i64 bitwise AND — decomposes to 4×i16 ANDs.
; CHECK-LABEL: and64:
; CHECK: rts
define i64 @and64(i64 %a, i64 %b) {
  %r = and i64 %a, %b
  ret i64 %r
}

; i64 bitwise OR.
; CHECK-LABEL: or64:
; CHECK: rts
define i64 @or64(i64 %a, i64 %b) {
  %r = or i64 %a, %b
  ret i64 %r
}

; i64 bitwise XOR.
; CHECK-LABEL: xor64:
; CHECK: rts
define i64 @xor64(i64 %a, i64 %b) {
  %r = xor i64 %a, %b
  ret i64 %r
}

; Zero-extend i32 to i64: low 4 bytes = input, high 4 bytes = 0.
; CHECK-LABEL: zext_i32_to_i64:
; CHECK: rts
define i64 @zext_i32_to_i64(i32 %v) {
  %r = zext i32 %v to i64
  ret i64 %r
}

; Truncate i64 to i32: extract low 4 bytes.
; CHECK-LABEL: trunc_i64_to_i32:
; CHECK: rts
define i32 @trunc_i64_to_i32(i64 %v) {
  %r = trunc i64 %v to i32
  ret i32 %r
}

; Return a 64-bit constant. Verifies sret return mechanism.
; CHECK-LABEL: return_const:
; CHECK: std
; CHECK: rts
define i64 @return_const() {
  ret i64 42
}

; Calling convention: i64 args all on stack (no piece in IX).
; The store to @g forces the value to be materialized.
; CHECK-LABEL: takes_i64:
; CHECK-NOT: tfr	x,d
; CHECK: rts
define void @takes_i64(i64 %x) {
  store volatile i64 %x, ptr @g
  ret void
}

; Calling convention: i16 in IX, i64 all on stack.
; CHECK-LABEL: i16_then_i64:
; CHECK: stx
; CHECK: rts
define void @i16_then_i64(i16 %a, i64 %b) {
  %ext = zext i16 %a to i64
  %sum = add i64 %ext, %b
  store volatile i64 %sum, ptr @g
  ret void
}
