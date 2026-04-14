; LLVM IR for ABI #3 validation tests: functions that take i32 args
; but do NOT return a long. Without the CCIfSplit rules in
; CC_MC6809, the first split-i16 piece of an i32 arg would land in
; IX whenever IX is free, and the gcc6809-compiled harness would
; not see it there. These tests catch that regression.

; Returns a 16-bit value extracted from a 32-bit arg. The synthetic
; sret IX is NOT in play because the return type fits in IX
; directly.
define dso_local i16 @low16(i32 noundef %x) local_unnamed_addr {
  %r = trunc i32 %x to i16
  ret i16 %r
}

define dso_local i16 @high16(i32 noundef %x) local_unnamed_addr {
  %hi = lshr i32 %x, 16
  %r = trunc i32 %hi to i16
  ret i16 %r
}

; Two i32 args + i16 return — exercises both args going to the
; stack and the second i32 not stomping the first via IX.
define dso_local i16 @diff_low16(i32 noundef %a, i32 noundef %b) local_unnamed_addr {
  %d = sub i32 %a, %b
  %r = trunc i32 %d to i16
  ret i16 %r
}

; Void-returning function that takes a long and stores it through
; a pointer. The pointer goes in IX (first ptr arg), the long
; goes on the stack as a 4-byte block.
define dso_local void @store_long(ptr noundef %dst, i32 noundef %val) local_unnamed_addr {
  store i32 %val, ptr %dst, align 1
  ret void
}
