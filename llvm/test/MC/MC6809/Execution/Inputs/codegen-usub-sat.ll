; LLVM IR functions for usub.sat (unsigned saturating subtract)
; execution tests. Bug #115: G_USUBSAT was lowered such that the
; carry-out from G_USUBO was not captured — codegen tested bit 0
; of the result high byte instead of the borrow flag, so usub_sat
; returned 0 for almost every input. That broke FORTIFY_SOURCE=3
; via __builtin_dynamic_object_size.

declare i8  @llvm.usub.sat.i8 (i8,  i8)
declare i16 @llvm.usub.sat.i16(i16, i16)

define dso_local i8 @usat_i8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = call i8 @llvm.usub.sat.i8(i8 %a, i8 %b)
  ret i8 %r
}

define dso_local i16 @usat_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = call i16 @llvm.usub.sat.i16(i16 %a, i16 %b)
  ret i16 %r
}
