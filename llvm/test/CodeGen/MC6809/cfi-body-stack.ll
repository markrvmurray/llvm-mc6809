; In a function without a frame pointer the CFA is S plus an offset, and the
; body moves S too: a temporary pushed and taken back (`pshs u ; addd ,s++`).
; Each such move is noted, so a debugger stopped between the push and its pop
; does not read the pushed value as the return address.  The prologue's and
; epilogue's own moves are described by them, once.  A block laid out after
; an epilogue that moved the CFA has it restated (CFIInstrInserter), and the
; result verifies: every block's incoming CFA matches every predecessor's.
;
; RUN: llc -mtriple=mc6809 -O2 -verify-cfiinstrs < %s | FileCheck %s

@sink = global i16 0
declare i16 @leaf(i16)
declare void @take(ptr, i16)

; CHECK-LABEL: middle:
; CHECK:       .cfi_def_cfa_offset 4
; CHECK:       pshs u
; CHECK-NEXT:  .cfi_adjust_cfa_offset 2
; CHECK-NEXT:  addd ,s++
; CHECK-NEXT:  .cfi_adjust_cfa_offset -2
; CHECK-NEXT:  tfr d,x
; CHECK-NEXT:  puls u,pc
define i16 @middle(i16 %a, i16 %b) uwtable {
  %s = add i16 %a, %b
  %r = call i16 @leaf(i16 %s)
  store i16 %r, ptr @sink
  %t = add i16 %r, %a
  ret i16 %t
}

; CHECK-LABEL: early:
; CHECK:       leas -10,s
; CHECK-NEXT:  .cfi_def_cfa_offset 16
; CHECK-NOT:   .cfi_adjust_cfa_offset
; CHECK:       leas 10,s
; CHECK-NEXT:  .cfi_def_cfa_offset 6
; CHECK-NEXT:  puls u,y,pc
; CHECK-NEXT:  .LBB1_2:
; CHECK-NEXT:  .cfi_def_cfa_offset 16
; CHECK-NEXT:  leas 10,s
define void @early(i16 %a) uwtable {
  %buf = alloca [4 x i16]
  call void @take(ptr %buf, i16 %a)
  %c = icmp eq i16 %a, 0
  br i1 %c, label %quick, label %slow
quick:
  ret void
slow:
  call void @take(ptr %buf, i16 %a)
  call void @take(ptr %buf, i16 %a)
  ret void
}
