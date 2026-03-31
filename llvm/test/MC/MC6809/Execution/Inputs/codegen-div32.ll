; LLVM IR functions for 32-bit division/remainder execution tests.

define dso_local i32 @test_udiv32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %r = udiv i32 %a, %b
  ret i32 %r
}

define dso_local i32 @test_sdiv32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %r = sdiv i32 %a, %b
  ret i32 %r
}

define dso_local i32 @test_urem32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %r = urem i32 %a, %b
  ret i32 %r
}

define dso_local i32 @test_srem32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %r = srem i32 %a, %b
  ret i32 %r
}
