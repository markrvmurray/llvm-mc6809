; LLVM IR functions for 32-bit multiply execution tests.

define dso_local i32 @test_mul32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %r = mul i32 %a, %b
  ret i32 %r
}

define dso_local i32 @test_mul32_const(i32 noundef %a) local_unnamed_addr {
entry:
  %r = mul i32 %a, 30000
  ret i32 %r
}
