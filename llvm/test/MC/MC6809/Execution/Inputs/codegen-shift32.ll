; LLVM IR functions for 32-bit shift execution tests.

define dso_local i32 @test_shl32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = shl i32 %a, %b
  ret i32 %r
}
define dso_local i32 @test_lshr32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = lshr i32 %a, %b
  ret i32 %r
}
define dso_local i32 @test_ashr32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = ashr i32 %a, %b
  ret i32 %r
}
