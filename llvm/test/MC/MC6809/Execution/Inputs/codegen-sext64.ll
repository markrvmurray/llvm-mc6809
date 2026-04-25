; LLVM IR functions for sext/zext to i64 execution tests.
;
; The existing test/CodeGen/MC6809/i64.ll only exercises ZEXT. This
; test fills the SEXT gap (sext i8/i16/i32 → i64) for both negative
; and positive operands so a sign-extension regression in the
; legalizer's i64-widening path surfaces immediately. Pairs with
; matching zext rows for sanity.

target triple = "mc6809-unknown-unknown"

define internal i64 @sext_i8 (i8  %x) { %r = sext i8  %x to i64 ret i64 %r }
define internal i64 @sext_i16(i16 %x) { %r = sext i16 %x to i64 ret i64 %r }
define internal i64 @sext_i32(i32 %x) { %r = sext i32 %x to i64 ret i64 %r }
define internal i64 @zext_i8 (i8  %x) { %r = zext i8  %x to i64 ret i64 %r }
define internal i64 @zext_i16(i16 %x) { %r = zext i16 %x to i64 ret i64 %r }
define internal i64 @zext_i32(i32 %x) { %r = zext i32 %x to i64 ret i64 %r }

define dso_local void @sext_i8_w (ptr %r, i8  %x) {
  %v = call i64 @sext_i8(i8 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
define dso_local void @sext_i16_w(ptr %r, i16 %x) {
  %v = call i64 @sext_i16(i16 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
define dso_local void @sext_i32_w(ptr %r, ptr %xp) {
  %x = load i32, ptr %xp, align 1
  %v = call i64 @sext_i32(i32 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
define dso_local void @zext_i8_w (ptr %r, i8  %x) {
  %v = call i64 @zext_i8(i8 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
define dso_local void @zext_i16_w(ptr %r, i16 %x) {
  %v = call i64 @zext_i16(i16 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
define dso_local void @zext_i32_w(ptr %r, ptr %xp) {
  %x = load i32, ptr %xp, align 1
  %v = call i64 @zext_i32(i32 %x)
  store i64 %v, ptr %r, align 1
  ret void
}
