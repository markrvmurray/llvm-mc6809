; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; HD6309 known-length memcpy with register (argument) pointers lowers to an
; inline TFM above a break-even length at speed levels, stays on the byte loop
; below it, and stays on the libcall under -Os.
target triple = "mc6809-unknown-unknown"

declare void @llvm.memcpy.p0.p0.i16(ptr, ptr, i16, i1)

; 32-byte register-pointer copy -> inline TFM.
define void @cp32(ptr %d, ptr %s) {
; CHECK-LABEL: cp32:
; CHECK: tfm x+,y+
  call void @llvm.memcpy.p0.p0.i16(ptr %d, ptr %s, i16 32, i1 false)
  ret void
}

; 8-byte register-pointer copy -> below break-even, no TFM.
define void @cp8(ptr %d, ptr %s) {
; CHECK-LABEL: cp8:
; CHECK-NOT: tfm
  call void @llvm.memcpy.p0.p0.i16(ptr %d, ptr %s, i16 8, i1 false)
  ret void
}

; 32-byte register-pointer copy under -Os -> shared libcall is smaller, no TFM.
define void @cp32_os(ptr %d, ptr %s) optsize {
; CHECK-LABEL: cp32_os:
; CHECK-NOT: tfm
  call void @llvm.memcpy.p0.p0.i16(ptr %d, ptr %s, i16 32, i1 false)
  ret void
}
