; LLVM IR functions for 32-bit add execution tests.

define dso_local i32 @add_i32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
entry:
  %add = add i32 %a, %b
  ret i32 %add
}

define dso_local i32 @add_i32_const(i32 noundef %a) local_unnamed_addr {
entry:
  %add = add i32 %a, 30000
  ret i32 %add
}

; ===================================================================
; gcc6809 ↔ LLVM-MC6809 long-return bridges
;
; Both compilers handle pointers identically (first ptr → IX), so
; the wrapper signature works even though the inner i32 conventions
; differ. The wrapper body is compiled by LLVM and uses LLVM's
; native i32 calling convention internally.
; ===================================================================

define dso_local void @add_i32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @add_i32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}

define dso_local void @add_i32_const_w(ptr noundef %out, i32 noundef %a) local_unnamed_addr {
  %r = call i32 @add_i32_const(i32 %a)
  store i32 %r, ptr %out, align 1
  ret void
}
