; Logical right shifts of an i32 by the amounts that only move bytes.

target triple = "mc6809-unknown-unknown"

define i32 @by16(i32 %a) {
  %r = lshr i32 %a, 16
  ret i32 %r
}

define i32 @by24(i32 %a) {
  %r = lshr i32 %a, 24
  ret i32 %r
}

define i8 @top_byte(i32 %a) {
  %s = lshr i32 %a, 24
  %r = trunc i32 %s to i8
  ret i8 %r
}
