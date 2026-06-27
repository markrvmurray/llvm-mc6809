; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s --check-prefixes=CHECK,HD6309
;
; A constant i16 shift by exactly one byte is a byte permute, not a runtime
; call. SHL by 8 is inlined on both CPUs; logical SHR by 8 is inlined on
; HD6309 (base 6809 keeps the libcall there because its high-byte->low-byte
; merge currently lowers through a stack spill). Sub-byte / arithmetic /
; variable shifts still libcall.

define i16 @shl8(i16 %x) {
; CHECK-LABEL: shl8:
; CHECK-NOT: __ashlhi3
entry:
  %r = shl i16 %x, 8
  ret i16 %r
}

define i16 @lshr8(i16 %x) {
; HD6309-LABEL: lshr8:
; HD6309-NOT: __lshrhi3
entry:
  %r = lshr i16 %x, 8
  ret i16 %r
}
