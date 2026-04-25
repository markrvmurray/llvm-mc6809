; LLVM IR functions for variable-count i64 shift execution tests.
;
; Routes to __ashldi3 / __lshrdi3 / __ashrdi3 (in shiftdi3.inc, the
; hand-asm runtime). add64's existing ashr64_w covers ashr only;
; this file widens to all three directions and varies the count
; through the off-by-one risk zones (0, 1, 7, 8, 31, 32, 63).

target triple = "mc6809-unknown-unknown"

define internal i64 @shl64 (i64 %a, i64 %n) { %r = shl  i64 %a, %n ret i64 %r }
define internal i64 @lshr64(i64 %a, i64 %n) { %r = lshr i64 %a, %n ret i64 %r }
define internal i64 @ashr64(i64 %a, i64 %n) { %r = ashr i64 %a, %n ret i64 %r }

define dso_local void @shl64_w(ptr %r, ptr %ap, i16 %count) {
  %a  = load i64, ptr %ap, align 1
  %n  = zext i16 %count to i64
  %x  = call i64 @shl64(i64 %a, i64 %n)
  store i64 %x, ptr %r, align 1
  ret void
}

define dso_local void @lshr64_w(ptr %r, ptr %ap, i16 %count) {
  %a  = load i64, ptr %ap, align 1
  %n  = zext i16 %count to i64
  %x  = call i64 @lshr64(i64 %a, i64 %n)
  store i64 %x, ptr %r, align 1
  ret void
}

define dso_local void @ashr64_w(ptr %r, ptr %ap, i16 %count) {
  %a  = load i64, ptr %ap, align 1
  %n  = zext i16 %count to i64
  %x  = call i64 @ashr64(i64 %a, i64 %n)
  store i64 %x, ptr %r, align 1
  ret void
}
