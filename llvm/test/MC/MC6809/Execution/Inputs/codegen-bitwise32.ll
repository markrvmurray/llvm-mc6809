; LLVM IR functions for 32-bit bitwise execution tests.

define dso_local i32 @test_and32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = and i32 %a, %b
  ret i32 %r
}
define dso_local i32 @test_or32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = or i32 %a, %b
  ret i32 %r
}
define dso_local i32 @test_xor32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = xor i32 %a, %b
  ret i32 %r
}
