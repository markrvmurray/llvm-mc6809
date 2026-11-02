; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 %s -o - | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s

; Moving the high half down needs no shift routine.

; CHECK-LABEL: by16:
; CHECK-NOT: __lshrsi3
define i32 @by16(i32 %a) {
  %r = lshr i32 %a, 16
  ret i32 %r
}

; CHECK-LABEL: by24:
; CHECK-NOT: __lshrsi3
define i32 @by24(i32 %a) {
  %r = lshr i32 %a, 24
  ret i32 %r
}

; CHECK-LABEL: top_byte:
; CHECK-NOT: __lshrsi3
define i8 @top_byte(i32 %a) {
  %s = lshr i32 %a, 24
  %r = trunc i32 %s to i8
  ret i8 %r
}

; Other amounts still call it.
; CHECK-LABEL: by20:
; CHECK: __lshrsi3
define i32 @by20(i32 %a) {
  %r = lshr i32 %a, 20
  ret i32 %r
}
