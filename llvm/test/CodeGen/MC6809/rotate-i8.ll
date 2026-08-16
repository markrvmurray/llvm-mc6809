; RUN: llc -mtriple=mc6809 -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs %s -o /dev/null
; RUN: llc -mtriple=mc6809 -O0 -verify-machineinstrs %s -o /dev/null

; The 6809 has no 8-bit rotate: ROL and ROR are 9-bit rotates through the
; carry flag. An 8-bit rotate therefore has to put the bit that wraps
; around into C first, and the instruction that does so exists only for
; its carry — its own byte result is dead. That producer must survive to
; the rotate, and the rotate must not be separated from it.
;
; The middle-end forms i8 rotates from `switch` range reduction (all case
; values even -> rotate right by one so odd inputs miss every case). It
; used to come out as a bare `rorb`, rotating in whatever the carry held,
; and every format specifier with a precision or length modifier in
; picolibc's vfprintf went to the default arm.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

declare i8 @llvm.fshl.i8(i8, i8, i8)
declare i8 @llvm.fshr.i8(i8, i8, i8)

; Rotate right by one: LSR on a copy puts bit 0 into C, ROR rotates it in
; at bit 7. The LSR result is dead; only its carry is wanted.
define i8 @ror1(i8 %x) {
; CHECK-LABEL: ror1:
; CHECK:         tfr b,a
; CHECK-NEXT:    lsra
; CHECK-NEXT:    rorb
; CHECK-NEXT:    rts
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 7)
  ret i8 %r
}

; Same rotate spelled with fshr.
define i8 @ror1_fshr(i8 %x) {
; CHECK-LABEL: ror1_fshr:
; CHECK:         tfr b,a
; CHECK-NEXT:    lsra
; CHECK-NEXT:    rorb
; CHECK-NEXT:    rts
  %r = call i8 @llvm.fshr.i8(i8 %x, i8 %x, i8 1)
  ret i8 %r
}

; Rotate left by one: shift, then add the bit that fell into the carry
; back in at bit 0.
define i8 @rol1(i8 %x) {
; CHECK-LABEL: rol1:
; CHECK:         aslb
; CHECK-NEXT:    adcb #0
; CHECK-NEXT:    rts
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 1)
  ret i8 %r
}

; Larger amounts are one-bit rotates chained; each step must be a real
; rotate, not a shift.
define i8 @rol3(i8 %x) {
; CHECK-LABEL: rol3:
; CHECK:         aslb
; CHECK-NEXT:    adcb #0
; CHECK-NEXT:    aslb
; CHECK-NEXT:    adcb #0
; CHECK-NEXT:    aslb
; CHECK-NEXT:    adcb #0
; CHECK-NEXT:    rts
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 3)
  ret i8 %r
}

; The shape SimplifyCFG's switch-range reduction produces: subtract the
; smallest case value, rotate right by one, dispatch. The rotate must not
; collapse to a bare ror.
define i8 @switch_rotate(i8 %c) {
; CHECK-LABEL: switch_rotate:
; CHECK:         addb #-42
; CHECK-NEXT:    tfr b,a
; CHECK-NEXT:    lsra
; CHECK-NEXT:    rorb
  %sub = add i8 %c, -42
  %rot = call i8 @llvm.fshl.i8(i8 %sub, i8 %sub, i8 7)
  switch i8 %rot, label %default [
    i8 0, label %star
    i8 2, label %dot
    i8 33, label %ell
  ]
star:
  ret i8 1
dot:
  ret i8 2
ell:
  ret i8 3
default:
  ret i8 0
}

