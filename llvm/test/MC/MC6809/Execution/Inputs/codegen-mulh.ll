; Wrappers for harness-mulh.c: the high half of a widening multiply, at each
; width the target handles.

target triple = "mc6809-unknown-unknown"

declare i8 @llvm.umulh.i8(i8, i8)
define i8 @umulh8(i8 %a, i8 %b) {
  %r = call i8 @llvm.umulh.i8(i8 %a, i8 %b)
  ret i8 %r
}

declare i8 @llvm.smulh.i8(i8, i8)
define i8 @smulh8(i8 %a, i8 %b) {
  %r = call i8 @llvm.smulh.i8(i8 %a, i8 %b)
  ret i8 %r
}

declare i16 @llvm.umulh.i16(i16, i16)
define i16 @umulh16(i16 %a, i16 %b) {
  %r = call i16 @llvm.umulh.i16(i16 %a, i16 %b)
  ret i16 %r
}

declare i16 @llvm.smulh.i16(i16, i16)
define i16 @smulh16(i16 %a, i16 %b) {
  %r = call i16 @llvm.smulh.i16(i16 %a, i16 %b)
  ret i16 %r
}

declare i32 @llvm.umulh.i32(i32, i32)
define i32 @umulh32(i32 %a, i32 %b) {
  %r = call i32 @llvm.umulh.i32(i32 %a, i32 %b)
  ret i32 %r
}

declare i32 @llvm.smulh.i32(i32, i32)
define i32 @smulh32(i32 %a, i32 %b) {
  %r = call i32 @llvm.smulh.i32(i32 %a, i32 %b)
  ret i32 %r
}
