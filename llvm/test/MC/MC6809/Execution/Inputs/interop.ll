define dso_local signext i8 @llvm_add8(i8 %a, i8 %b) {
  %r = add i8 %a, %b
  ret i8 %r
}
define dso_local signext i8 @llvm_negate8(i8 %a) {
  %r = sub i8 0, %a
  ret i8 %r
}
define dso_local i16 @llvm_identity16(i16 %a) {
  ret i16 %a
}
