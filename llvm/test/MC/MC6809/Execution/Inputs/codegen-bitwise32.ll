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

; ===================================================================
; gcc6809 ↔ LLVM-MC6809 long-return bridges
;
; Both compilers handle pointers identically (first ptr → IX), so
; the wrapper signature works even though the inner i32 conventions
; differ. The wrapper bodies are compiled by LLVM and use LLVM's
; native i32 calling convention internally.
; ===================================================================

define dso_local void @test_and32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_and32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
define dso_local void @test_or32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_or32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
define dso_local void @test_xor32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_xor32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
