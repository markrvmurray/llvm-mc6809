; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 -stop-after=instruction-select %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 -mcpu=hd6309 -filetype=null %s

; -Og keeps variables alive with llvm.fake.use. A double or a long long has no
; register class, so its use goes away; narrower values keep theirs.

declare void @llvm.fake.use(...)

; CHECK-LABEL: name: wide
; CHECK-NOT: FAKE_USE
; CHECK: RTS
define i16 @wide(double %d, i64 %l) {
  notail call void (...) @llvm.fake.use(double %d)
  notail call void (...) @llvm.fake.use(i64 %l)
  ret i16 0
}

; CHECK-LABEL: name: narrow
; CHECK: FAKE_USE
define i16 @narrow(i16 %a) {
  notail call void (...) @llvm.fake.use(i16 %a)
  ret i16 0
}
