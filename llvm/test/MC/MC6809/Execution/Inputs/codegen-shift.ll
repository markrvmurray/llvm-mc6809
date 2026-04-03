; LLVM IR functions for shift operation execution tests (constant shifts).
; Variable shifts require libcall functions (__ashlqi3 etc.) not yet provided.

define dso_local i8 @test_shl8_1(i8 noundef %a) local_unnamed_addr {
  %r = shl i8 %a, 1
  ret i8 %r
}
define dso_local i8 @test_shl8_4(i8 noundef %a) local_unnamed_addr {
  %r = shl i8 %a, 4
  ret i8 %r
}
define dso_local i8 @test_lshr8_1(i8 noundef %a) local_unnamed_addr {
  %r = lshr i8 %a, 1
  ret i8 %r
}
define dso_local i8 @test_lshr8_4(i8 noundef %a) local_unnamed_addr {
  %r = lshr i8 %a, 4
  ret i8 %r
}
define dso_local i8 @test_ashr8_1(i8 noundef signext %a) local_unnamed_addr {
  %r = ashr i8 %a, 1
  ret i8 %r
}
define dso_local i8 @test_ashr8_4(i8 noundef signext %a) local_unnamed_addr {
  %r = ashr i8 %a, 4
  ret i8 %r
}

; i16 constant shifts
define dso_local i16 @test_shl16_1(i16 noundef %a) local_unnamed_addr {
  %r = shl i16 %a, 1
  ret i16 %r
}
define dso_local i16 @test_lshr16_1(i16 noundef %a) local_unnamed_addr {
  %r = lshr i16 %a, 1
  ret i16 %r
}
define dso_local i16 @test_ashr16_1(i16 noundef %a) local_unnamed_addr {
  %r = ashr i16 %a, 1
  ret i16 %r
}
define dso_local i16 @test_lshr16_4(i16 noundef %a) local_unnamed_addr {
  %r = lshr i16 %a, 4
  ret i16 %r
}
define dso_local i16 @test_ashr16_4(i16 noundef %a) local_unnamed_addr {
  %r = ashr i16 %a, 4
  ret i16 %r
}

; Variable shifts (require libcall functions)
define dso_local i8 @test_shl8_var(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
  %r = shl i8 %a, %b
  ret i8 %r
}
define dso_local i8 @test_lshr8_var(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
  %r = lshr i8 %a, %b
  ret i8 %r
}
define dso_local i8 @test_ashr8_var(i8 noundef signext %a, i8 noundef %b) local_unnamed_addr {
  %r = ashr i8 %a, %b
  ret i8 %r
}
define dso_local i16 @test_shl16_var(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %r = shl i16 %a, %b
  ret i16 %r
}
define dso_local i16 @test_lshr16_var(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %r = lshr i16 %a, %b
  ret i16 %r
}
define dso_local i16 @test_ashr16_var(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %r = ashr i16 %a, %b
  ret i16 %r
}
