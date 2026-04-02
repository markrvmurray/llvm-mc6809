; LLVM IR functions for bitwise operation execution tests (i8 and i16).

define dso_local i8 @test_and8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = and i8 %a, %b
  ret i8 %r
}

define dso_local i16 @test_and16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = and i16 %a, %b
  ret i16 %r
}

define dso_local i8 @test_or8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = or i8 %a, %b
  ret i8 %r
}

define dso_local i16 @test_or16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = or i16 %a, %b
  ret i16 %r
}

define dso_local i8 @test_xor8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = xor i8 %a, %b
  ret i8 %r
}

define dso_local i16 @test_xor16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = xor i16 %a, %b
  ret i16 %r
}
