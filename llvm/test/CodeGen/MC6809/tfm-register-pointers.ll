; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; HD6309 known-length memcpy with register (argument) pointers lowers to an
; inline TFM via the order-free BlockCopy_* path: the pointers are handed to
; regalloc in the {IX,IY} class and the postbyte records whichever assignment it
; picked, so a dst argument already in X needs no `tfr x,y` swap. Under -Os the
; shared memcpy libcall is smaller, so it stays a call there.
target triple = "mc6809-unknown-unknown"

declare void @llvm.memcpy.p0.p0.i16(ptr, ptr, i16, i1)

; 32-byte register-pointer copy -> inline TFM, no forced swap (regalloc chooses
; the IX/IY assignment; the postbyte order follows it).
define void @cp32(ptr %d, ptr %s) {
; CHECK-LABEL: cp32:
; CHECK: tfm {{[xy]}}+,{{[xy]}}+
; CHECK-NOT: tfr {{[xy]}},{{[xy]}}
  call void @llvm.memcpy.p0.p0.i16(ptr %d, ptr %s, i16 32, i1 false)
  ret void
}

; 8-byte register-pointer copy also inlines: with no swap penalty the order-free
; path has no raised break-even, so the normal length minimum applies.
define void @cp8(ptr %d, ptr %s) {
; CHECK-LABEL: cp8:
; CHECK: tfm {{[xy]}}+,{{[xy]}}+
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
