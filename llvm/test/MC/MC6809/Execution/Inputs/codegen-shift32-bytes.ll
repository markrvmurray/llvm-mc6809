; 32-bit shifts by a whole number of bytes, which the 6809 does inline as
; byte moves (the 6309 still calls the library). clang -O2 output for
; shl/lshr/ashr by 8, 16 and 24 of a uint32_t or int32_t.

target triple = "mc6809-unknown-unknown"


define noundef i32 @shl8(i32 noundef %a) {
entry:
  %shl = shl i32 %a, 8
  ret i32 %shl
}

define noundef i32 @shl16(i32 noundef %a) {
entry:
  %shl = shl i32 %a, 16
  ret i32 %shl
}

define noundef i32 @shl24(i32 noundef %a) {
entry:
  %shl = shl i32 %a, 24
  ret i32 %shl
}

define noundef i32 @lshr8(i32 noundef %a) {
entry:
  %shr = lshr i32 %a, 8
  ret i32 %shr
}

define noundef i32 @lshr16(i32 noundef %a) {
entry:
  %shr = lshr i32 %a, 16
  ret i32 %shr
}

define noundef i32 @lshr24(i32 noundef %a) {
entry:
  %shr = lshr i32 %a, 24
  ret i32 %shr
}

define noundef i32 @ashr8(i32 noundef %a) {
entry:
  %shr = ashr i32 %a, 8
  ret i32 %shr
}

define noundef i32 @ashr16(i32 noundef %a) {
entry:
  %shr = ashr i32 %a, 16
  ret i32 %shr
}

define noundef i32 @ashr24(i32 noundef %a) {
entry:
  %shr = ashr i32 %a, 24
  ret i32 %shr
}

