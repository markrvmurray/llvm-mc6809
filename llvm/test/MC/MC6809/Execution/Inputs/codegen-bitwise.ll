; LLVM IR functions for bitwise operation execution tests.
; NOTE: i16 bitwise ops currently emit 6309-only ANDD/ORD/EORD on 6809
; (bug #26). Only i8 ops are tested here until that's fixed.

define dso_local i8 @test_and8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = and i8 %a, %b
  ret i8 %r
}

define dso_local i8 @test_or8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = or i8 %a, %b
  ret i8 %r
}

define dso_local i8 @test_xor8(i8 noundef %a, i8 noundef %b) local_unnamed_addr {
entry:
  %r = xor i8 %a, %b
  ret i8 %r
}
