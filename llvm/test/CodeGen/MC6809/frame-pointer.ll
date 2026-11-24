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

; Without a frame pointer the CFA stays S-relative: each push and the frame's
; allocation move it, the prologue says where U and Y went, and the epilogue
; follows it back down. The return is folded into the last pull, so nothing
; follows that.
; CHECK-LABEL: unwinds:
; CHECK:       .cfi_def_cfa s, 2
; CHECK:       pshs u,y
; CHECK-NEXT:  .cfi_adjust_cfa_offset 4
; CHECK-NEXT:  .cfi_rel_offset u, 2
; CHECK-NEXT:  .cfi_rel_offset y, 0
; CHECK-NEXT:  leas -10,s
; CHECK-NEXT:  .cfi_adjust_cfa_offset 10
; CHECK:       leas 10,s
; CHECK-NEXT:  .cfi_adjust_cfa_offset -10
; CHECK-NEXT:  puls u,y,pc
; CHECK-NEXT:  .Lfunc_end
define void @unwinds(i16 %n) uwtable {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %n)
  call void @take(ptr %buf, i16 %n)
  ret void
}

; With a frame pointer the locals are allocated before U is pushed, so U's
; slot is below them (CFA-12, not CFA-4); the CFA moves to U at `tfr s,u`,
; where S may then move as it likes, back to S at `tfr u,s`, and U is the
; caller's again once pulled.
; CHECK-LABEL: framed:
; CHECK:       leas -8,s
; CHECK-NEXT:  .cfi_adjust_cfa_offset 8
; CHECK-NEXT:  pshs u
; CHECK-NEXT:  .cfi_adjust_cfa_offset 2
; CHECK-NEXT:  .cfi_rel_offset u, 0
; CHECK-NEXT:  tfr s,u
; CHECK-NEXT:  .cfi_def_cfa_register u
; CHECK-NEXT:  leas -2,s
; CHECK:       tfr u,s
; CHECK-NEXT:  .cfi_def_cfa_register s
; CHECK-NEXT:  puls u
; CHECK-NEXT:  .cfi_adjust_cfa_offset -2
; CHECK-NEXT:  .cfi_restore u
; CHECK-NEXT:  leas 8,s
; CHECK-NEXT:  .cfi_adjust_cfa_offset -8
; CHECK-NEXT:  rts
define void @framed(i16 %n) "frame-pointer"="all" uwtable {
  %buf = alloca [4 x i16]
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
