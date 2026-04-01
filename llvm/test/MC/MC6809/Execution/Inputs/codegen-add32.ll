; LLVM IR functions for 32-bit add execution tests.

define dso_local i32 @add_i32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %add = add i32 %a, %b
  ret i32 %add
}

define dso_local i32 @add_i32_const(i32 noundef %a) local_unnamed_addr {
entry:
  %add = add i32 %a, 30000
  ret i32 %add
}
