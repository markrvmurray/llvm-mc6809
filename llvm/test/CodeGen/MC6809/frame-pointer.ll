; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O0 < %s | FileCheck %s

; A frame is addressed from S. U is set up as a frame pointer only when
; something needs a fixed base -- a variable-sized object, or a taken frame
; address -- or when one is asked for ("frame-pointer"="all", which clang
; emits for -fno-omit-frame-pointer). Making a call is not a reason: the
; outgoing arguments go in an area reserved in the frame, so S does not move
; around a call.

target triple = "mc6809-unknown-unknown"

declare void @take(ptr, i16)

; A function that calls, with locals and outgoing arguments: no frame
; pointer, no stack adjustment around the calls.
; CHECK-LABEL: calls:
; CHECK-NOT:   tfr s,u
; CHECK:       lbsr take
; CHECK-NOT:   leas
; CHECK:       lbsr take
; CHECK-NOT:   tfr u,s
; CHECK:       {{rts|puls .*,pc}}
define void @calls(i16 %n) {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %n)
  call void @take(ptr %buf, i16 %n)
  ret void
}

; A variable-sized object moves S by a run-time amount: U holds the frame.
; CHECK-LABEL: dynamic:
; CHECK:       pshs u
; CHECK:       tfr s,u
; CHECK:       tfr u,s
; CHECK:       puls u
define void @dynamic(i16 %n) {
  %buf = alloca i8, i16 %n
  call void @take(ptr %buf, i16 %n)
  ret void
}

; Asked for one: kept, even in a function that needs none.
; CHECK-LABEL: asked:
; CHECK:       pshs u
; CHECK:       tfr s,u
; CHECK:       lbsr take
; CHECK:       tfr u,s
; CHECK:       puls u
define void @asked(i16 %n) "frame-pointer"="all" {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %n)
  ret void
}

; Without a frame pointer the CFA stays S-relative; the prologue says where
; it saved U and Y, how far S has moved after the pushes and after the frame
; is allocated, and the epilogue follows it back down. The return is folded
; into the last pull, so nothing follows that.
; CHECK-LABEL: unwinds:
; CHECK:       .cfi_def_cfa s, 2
; CHECK:       pshs u,y
; CHECK-NEXT:  .cfi_offset u, -4
; CHECK-NEXT:  .cfi_offset y, -6
; CHECK-NEXT:  .cfi_def_cfa_offset 6
; CHECK-NEXT:  leas -10,s
; CHECK-NEXT:  .cfi_def_cfa_offset 16
; CHECK:       leas 10,s
; CHECK-NEXT:  .cfi_def_cfa_offset 6
; CHECK-NEXT:  puls u,y,pc
; CHECK-NEXT:  .Lfunc_end
define void @unwinds(i16 %n) uwtable {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %n)
  call void @take(ptr %buf, i16 %n)
  ret void
}

; Asked for one in a function whose body never touches U: still kept.
; CHECK-LABEL: asked_bare:
; CHECK:       pshs u
; CHECK:       tfr s,u
; CHECK:       lbsr take
; CHECK:       tfr u,s
; CHECK:       puls u,pc
define void @asked_bare() "frame-pointer"="all" {
  call void @take(ptr null, i16 0)
  ret void
}
