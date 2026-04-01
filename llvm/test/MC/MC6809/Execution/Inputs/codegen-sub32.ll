; LLVM IR functions for 32-bit sub execution tests.

define dso_local i32 @sub_i32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %sub = sub i32 %a, %b
  ret i32 %sub
}
