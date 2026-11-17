; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -mc6809-enable-spill-opt=false < %s | FileCheck %s

; Every volatile access reaches the assembly. A device register may clear a
; flag on each read or need a value written twice, so the clean-up passes that
; remove a repeated load or an overwritten store must leave these alone.

define i8 @read_twice(ptr %p) {
; CHECK-LABEL: read_twice:
; CHECK:       ldb ,x
; CHECK-NEXT:  ldb ,x
; CHECK-NEXT:  {{rts|puls .*,pc}}
  %discard = load volatile i8, ptr %p
  %v = load volatile i8, ptr %p
  ret i8 %v
}

define void @write_twice(ptr %p, i8 %v) {
; CHECK-LABEL: write_twice:
; CHECK:       stb ,x
; CHECK-NEXT:  stb ,x
; CHECK-NEXT:  {{rts|puls .*,pc}}
  store volatile i8 %v, ptr %p
  store volatile i8 %v, ptr %p
  ret void
}

define i16 @read_twice16(ptr %p) {
; CHECK-LABEL: read_twice16:
; CHECK:       ld{{[dxy]}} ,x
; CHECK-NEXT:  ld{{[dxy]}} ,x
; CHECK:       {{rts|puls .*,pc}}
  %discard = load volatile i16, ptr %p
  %v = load volatile i16, ptr %p
  ret i16 %v
}

define void @write_twice16(ptr %p, i16 %v) {
; CHECK-LABEL: write_twice16:
; CHECK:       st{{[dxy]}} ,x
; CHECK-NEXT:  st{{[dxy]}} ,x
; CHECK:       {{rts|puls .*,pc}}
  store volatile i16 %v, ptr %p
  store volatile i16 %v, ptr %p
  ret void
}

; A volatile write between two ordinary ones: the first ordinary store is not
; dead, and the value is not assumed to still be in the register afterwards.
define i8 @write_then_read(ptr %p, i8 %v) {
; CHECK-LABEL: write_then_read:
; CHECK:       stb ,x
; CHECK-NEXT:  ldb ,x
; CHECK-NEXT:  {{rts|puls .*,pc}}
  store volatile i8 %v, ptr %p
  %r = load volatile i8, ptr %p
  ret i8 %r
}

; The ordinary case still folds: nothing here is volatile.
define void @plain_write_twice(ptr %p, i8 %v) {
; CHECK-LABEL: plain_write_twice:
; CHECK:       stb ,x
; CHECK-NEXT:  {{rts|puls .*,pc}}
  store i8 %v, ptr %p
  store i8 %v, ptr %p
  ret void
}
