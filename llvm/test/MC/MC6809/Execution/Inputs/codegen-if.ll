; LLVM IR functions for conditional (if/select) execution tests.

; Signed max (sgt): return a > b ? a : b
define dso_local i8 @test_max_s8(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
  %cmp = icmp sgt i8 %a, %b
  %r = select i1 %cmp, i8 %a, i8 %b
  ret i8 %r
}

define dso_local i16 @test_max_s16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp sgt i16 %a, %b
  %r = select i1 %cmp, i16 %a, i16 %b
  ret i16 %r
}

; Unsigned max (ugt): return a > b ? a : b
define dso_local i8 @test_max_u8(i8 noundef zeroext %a, i8 noundef zeroext %b) local_unnamed_addr {
  %cmp = icmp ugt i8 %a, %b
  %r = select i1 %cmp, i8 %a, i8 %b
  ret i8 %r
}

define dso_local i16 @test_max_u16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp ugt i16 %a, %b
  %r = select i1 %cmp, i16 %a, i16 %b
  ret i16 %r
}

; Signed min (slt): return a < b ? a : b
define dso_local i8 @test_min_s8(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
  %cmp = icmp slt i8 %a, %b
  %r = select i1 %cmp, i8 %a, i8 %b
  ret i8 %r
}

; Equality: return a == b ? 1 : 0
define dso_local i8 @test_eq8(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
  %cmp = icmp eq i8 %a, %b
  %r = zext i1 %cmp to i8
  ret i8 %r
}

; Not-equal: return a != b ? 1 : 0
define dso_local i8 @test_ne8(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
  %cmp = icmp ne i8 %a, %b
  %r = zext i1 %cmp to i8
  ret i8 %r
}
