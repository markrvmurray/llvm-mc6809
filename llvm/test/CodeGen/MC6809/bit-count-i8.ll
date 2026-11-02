; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; A bit count of an i8 goes through the 16-bit routine: there is no 8-bit one.

declare i8 @llvm.ctlz.i8(i8, i1)
declare i8 @llvm.cttz.i8(i8, i1)
declare i8 @llvm.ctpop.i8(i8)

; CHECK-LABEL: clz8:
; CHECK: __clzhi2
define i8 @clz8(i8 %a) {
  %r = call i8 @llvm.ctlz.i8(i8 %a, i1 false)
  ret i8 %r
}

; CHECK-LABEL: ctz8:
; CHECK: __ctzhi2
define i8 @ctz8(i8 %a) {
  %r = call i8 @llvm.cttz.i8(i8 %a, i1 false)
  ret i8 %r
}

; CHECK-LABEL: pop8:
; CHECK: __popcounthi2
define i8 @pop8(i8 %a) {
  %r = call i8 @llvm.ctpop.i8(i8 %a)
  ret i8 %r
}
