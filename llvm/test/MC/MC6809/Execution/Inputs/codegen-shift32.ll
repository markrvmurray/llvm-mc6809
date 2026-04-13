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

; ===================================================================
; gcc6809 ↔ LLVM-MC6809 long-return bridges
;
; Both compilers handle pointers identically (first ptr → IX), so
; the wrapper signature works even though the inner i32 conventions
; differ. The wrapper bodies are compiled by LLVM and use LLVM's
; native i32 calling convention internally.
; ===================================================================

define dso_local void @test_shl32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_shl32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
define dso_local void @test_lshr32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_lshr32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
define dso_local void @test_ashr32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_ashr32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}
