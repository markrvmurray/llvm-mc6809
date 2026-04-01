; LLVM IR functions for 16-bit add execution tests.

define dso_local i16 @add_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %add = add i16 %a, %b
  ret i16 %add
}

define dso_local i16 @add_i16_const(i16 noundef %a) local_unnamed_addr {
entry:
  %add = add i16 %a, 1000
  ret i16 %add
}

define dso_local i16 @add_i16_chain(i16 noundef %a, i16 noundef %b, i16 noundef %c) local_unnamed_addr {
entry:
  %add1 = add i16 %a, %b
  %add2 = add i16 %add1, %c
  ret i16 %add2
}
