; LLVM IR function for 16-bit multiply execution test.

define dso_local i16 @test_mul16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %mul = mul i16 %a, %b
  ret i16 %mul
}
