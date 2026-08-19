; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: neither builtin used to compile.  Reversing the bits of a
; value has no instruction and no libcall (LLVM has no RTLIB entry for it),
; so it is expanded in place; swapping the bytes of a 16-bit value is
; swapping the halves of the accumulator, which the byte pseudos do -- it
; was marked legal but nothing selected it, so any use of it (htons, say)
; failed to compile.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

declare i8 @llvm.bitreverse.i8(i8)
declare i16 @llvm.bitreverse.i16(i16)
declare i16 @llvm.bswap.i16(i16)

; CHECK-LABEL: rev8:
; CHECK-NOT: lbsr
; CHECK: rts
define i8 @rev8(i8 %x) {
  %r = call i8 @llvm.bitreverse.i8(i8 %x)
  ret i8 %r
}

; CHECK-LABEL: rev16:
; CHECK-NOT: lbsr
; CHECK: rts
define i16 @rev16(i16 %x) {
  %r = call i16 @llvm.bitreverse.i16(i16 %x)
  ret i16 %r
}

; CHECK-LABEL: swap16:
; CHECK-NOT: lbsr
; CHECK: rts
define i16 @swap16(i16 %x) {
  %r = call i16 @llvm.bswap.i16(i16 %x)
  ret i16 %r
}
