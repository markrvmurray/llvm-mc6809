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
; Two CC differences still need bridging:
;
;   1. long return: gcc6809 uses an implicit out-pointer in X but
;      LLVM splits the result across X (high) + first stack arg slot
;      (low). Wrap with a void-returning out-pointer signature.
;
;   2. i32 args: gcc6809 passes a `long` arg as a single 4-byte
;      stack block. LLVM-MC6809 splits each i32 into 2 i16 halves
;      and applies the i16 CC to each (so a's high half goes in IX
;      if no earlier ptr/i16 arg has consumed it). When the layouts
;      don't line up, pass the longs by *pointer* instead.
;
; Both wrapper bodies are compiled by LLVM, so the inner calls use
; LLVM's native CC. Only the wrapper *signature* needs to be in a
; form gcc6809 can handle.
;
; (Historical note: i8 stack args used to need bridging too. The
; LLVM CC was updated to use 1-byte slots matching gcc6809 — see
; MC6809CallingConv.td. The i8 functions in this file are now
; called directly from the harness.)
; ===================================================================

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
