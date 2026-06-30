; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; HD6309 compile-time-absolute (global) memcpy / memmove / memset all lower to an
; inline TFM through the one unified BlockCopy_* path: each pointer is handed to
; regalloc in the {IX,IY} class and the postbyte records whichever assignment it
; picked, so the global materializes straight into its index register with no
; `tfr x,y` shuffle. The transfer mode is what differs per intrinsic.
target triple = "mc6809-unknown-unknown"

@g1 = external global [64 x i8]

declare void @llvm.memcpy.p0.p0.i16(ptr, ptr, i16, i1)
declare void @llvm.memmove.p0.p0.i16(ptr, ptr, i16, i1)
declare void @llvm.memset.p0.i16(ptr, i8, i16, i1)

; Ascending copy -> TFM0pp (post-increment both).
define i8 @gcpy(ptr %s) {
; CHECK-LABEL: gcpy:
; CHECK: tfm {{[xy]}}+,{{[xy]}}+
; CHECK-NOT: tfr {{[xy]}},{{[xy]}}
  call void @llvm.memcpy.p0.p0.i16(ptr @g1, ptr %s, i16 16, i1 false)
  %p = getelementptr [64 x i8], ptr @g1, i16 0, i16 5
  %v = load volatile i8, ptr %p
  ret i8 %v
}

; Overlapping same-object memmove with dst above src -> descending TFM1pp
; (post-decrement both, started from the high end).
define i8 @gmov() {
; CHECK-LABEL: gmov:
; CHECK: tfm {{[xy]}}-,{{[xy]}}-
  %d = getelementptr [64 x i8], ptr @g1, i16 0, i16 1
  call void @llvm.memmove.p0.p0.i16(ptr %d, ptr @g1, i16 16, i1 false)
  %p = getelementptr [64 x i8], ptr @g1, i16 0, i16 5
  %v = load volatile i8, ptr %p
  ret i8 %v
}

; memset fills via TFM3pp (source stays on the just-stored fill byte while the
; destination sweeps the range).
define i8 @gset() optsize {
; CHECK-LABEL: gset:
; CHECK: tfm {{[xy]}},{{[xy]}}+
  call void @llvm.memset.p0.i16(ptr @g1, i8 65, i16 16, i1 false)
  %p = getelementptr [64 x i8], ptr @g1, i16 0, i16 5
  %v = load volatile i8, ptr %p
  ret i8 %v
}
