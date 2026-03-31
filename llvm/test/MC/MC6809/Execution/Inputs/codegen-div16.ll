; LLVM IR functions for 16-bit division/remainder execution tests.

define dso_local i16 @test_udiv16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = udiv i16 %a, %b
  ret i16 %r
}

define dso_local i16 @test_sdiv16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = sdiv i16 %a, %b
  ret i16 %r
}

define dso_local i16 @test_urem16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = urem i16 %a, %b
  ret i16 %r
}

define dso_local i16 @test_srem16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
entry:
  %r = srem i16 %a, %b
  ret i16 %r
}
