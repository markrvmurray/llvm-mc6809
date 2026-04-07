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


; eq/ne returning i16 (zext i1 → i16)
define dso_local i16 @test_eq16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %c = icmp eq i16 %a, %b
  %r = zext i1 %c to i16
  ret i16 %r
}

define dso_local i16 @test_ne16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %c = icmp ne i16 %a, %b
  %r = zext i1 %c to i16
  ret i16 %r
}

; i32 signed max: return a > b ? a : b
define dso_local i32 @test_max_s32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %cmp = icmp sgt i32 %a, %b
  %r = select i1 %cmp, i32 %a, i32 %b
  ret i32 %r
}

; i32 unsigned min: return a < b ? a : b
define dso_local i32 @test_min_u32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %cmp = icmp ult i32 %a, %b
  %r = select i1 %cmp, i32 %a, i32 %b
  ret i32 %r
}

; i32 equality: return a == b ? 1 : 0
define dso_local i16 @test_eq32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %c = icmp eq i32 %a, %b
  %r = zext i1 %c to i16
  ret i16 %r
}

; i32 not-equal: return a != b ? 1 : 0
define dso_local i16 @test_ne32(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %c = icmp ne i32 %a, %b
  %r = zext i1 %c to i16
  ret i16 %r
}

; ===================================================================
; gcc6809 ↔ LLVM-MC6809 calling-convention bridges
;
; Two CC differences need bridging:
;
;   1. i8 stack arg layout: gcc6809 emits a 1-byte slot via `stb ,-s`
;      and reads from offset 0, but LLVM-MC6809 reserves a 2-byte
;      slot with the byte at offset 1 (CCAssignToStack<2,2> in
;      CC_MC6809; see MC6809OutgoingArgsHandler::getStackAddress for
;      the +1 byte adjust). The first i8 arg in B is fine — both
;      compilers agree on B for the first i8 — but the *second* i8
;      arg (and beyond) lands at the wrong offset.
;      Fix: wrap each i8-taking function with an i16-taking wrapper.
;      i16 stack arg passing matches between the two compilers.
;
;   2. long return: gcc6809 uses an implicit out-pointer in X but
;      LLVM splits the result across X (high) + first stack arg slot
;      (low). Wrap with a void-returning out-pointer signature.
;
; Both wrapper bodies are compiled by LLVM, so the inner calls use
; LLVM's native CC. Only the wrapper *signature* needs to be in a
; form gcc6809 can handle.
; ===================================================================

; --- i8 stack-arg bridges (i16-taking wrappers around i8 functions) ---

define dso_local i16 @test_max_s8_w(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %a8 = trunc i16 %a to i8
  %b8 = trunc i16 %b to i8
  %r8 = call signext i8 @test_max_s8(i8 signext %a8, i8 signext %b8)
  %r = sext i8 %r8 to i16
  ret i16 %r
}

define dso_local i16 @test_max_u8_w(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %a8 = trunc i16 %a to i8
  %b8 = trunc i16 %b to i8
  %r8 = call zeroext i8 @test_max_u8(i8 zeroext %a8, i8 zeroext %b8)
  %r = zext i8 %r8 to i16
  ret i16 %r
}

define dso_local i16 @test_min_s8_w(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %a8 = trunc i16 %a to i8
  %b8 = trunc i16 %b to i8
  %r8 = call signext i8 @test_min_s8(i8 signext %a8, i8 signext %b8)
  %r = sext i8 %r8 to i16
  ret i16 %r
}

define dso_local i16 @test_eq8_w(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %a8 = trunc i16 %a to i8
  %b8 = trunc i16 %b to i8
  %r8 = call zeroext i8 @test_eq8(i8 signext %a8, i8 signext %b8)
  %r = zext i8 %r8 to i16
  ret i16 %r
}

define dso_local i16 @test_ne8_w(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %a8 = trunc i16 %a to i8
  %b8 = trunc i16 %b to i8
  %r8 = call zeroext i8 @test_ne8(i8 signext %a8, i8 signext %b8)
  %r = zext i8 %r8 to i16
  ret i16 %r
}

; --- long-return bridges (out-pointer-taking wrappers) ---
;
; Note: these have THREE i32-or-larger args (out, a, b) but the
; first ptr `out` consumes IX. The remaining 8 bytes (2 longs) end
; up entirely on the stack from both compilers' perspectives, with
; matching big-endian byte layout. So the wrapper signature works
; even though the inner i32 conventions differ.

define dso_local void @test_max_s32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_max_s32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}

define dso_local void @test_min_u32_w(ptr noundef %out, i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %r = call i32 @test_min_u32(i32 %a, i32 %b)
  store i32 %r, ptr %out, align 1
  ret void
}

; --- i32-arg + i16-return bridges (pointer-to-long inputs) ---
;
; gcc6809 passes a `long` arg as a single 4-byte block on the stack.
; LLVM-MC6809 splits each i32 into 2 i16 halves and applies the i16
; CC to each (so a_hi → IX if there's no other i16/ptr arg, then
; everything else → stack). The two layouts are NOT equivalent: from
; gcc6809's perspective the bytes are at stack[0..7]; from LLVM's
; perspective IX has the first i16 (= a's high half) and only 6 bytes
; are on the stack.
;
; Workaround: pass the longs by *pointer*. Both compilers agree on
; pointer args (first ptr → IX, subsequent ptrs → stack), so the
; wrapper signature is bridge-compatible. The wrapper body
; dereferences and calls the real function via LLVM's native CC.

define dso_local i16 @test_eq32_w(ptr noundef %ap, ptr noundef %bp) local_unnamed_addr {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %r = call i16 @test_eq32(i32 %a, i32 %b)
  ret i16 %r
}

define dso_local i16 @test_ne32_w(ptr noundef %ap, ptr noundef %bp) local_unnamed_addr {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %r = call i16 @test_ne32(i32 %a, i32 %b)
  ret i16 %r
}
