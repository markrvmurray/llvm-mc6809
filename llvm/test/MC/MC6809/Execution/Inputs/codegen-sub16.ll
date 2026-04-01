; LLVM IR functions for 16-bit sub execution tests.

define dso_local i16 @sub_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %sub = sub i16 %a, %b
  ret i16 %sub
}

define dso_local i16 @sub_i16_const(i16 noundef %a) local_unnamed_addr {
entry:
  %sub = sub i16 %a, 100
  ret i16 %sub
}
