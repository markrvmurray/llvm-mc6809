; LLVM IR functions for 64-bit add execution tests.
;
; gcc6809's long long is 4 bytes (non-conformant), so the C harness
; can't pass/return i64 directly. Instead, the harness calls pointer-
; based wrappers (_w suffix) that load/store i64 through pointers.
; The actual add64 function is a normal i64 → i64 operation.

target triple = "mc6809-unknown-unknown"

define internal i64 @add64(i64 %a, i64 %b) {
  %r = add i64 %a, %b
  ret i64 %r
}

define internal i64 @sub64(i64 %a, i64 %b) {
  %r = sub i64 %a, %b
  ret i64 %r
}

; Wrapper: void add64_w(i8* result, i8* a, i8* b)
; Loads two i64 values from a and b, adds them, stores to result.
define dso_local void @add64_w(ptr %result, ptr %a, ptr %b) {
  %av = load i64, ptr %a, align 1
  %bv = load i64, ptr %b, align 1
  %r = call i64 @add64(i64 %av, i64 %bv)
  store i64 %r, ptr %result, align 1
  ret void
}

; Wrapper: void sub64_w(i8* result, i8* a, i8* b)
define dso_local void @sub64_w(ptr %result, ptr %a, ptr %b) {
  %av = load i64, ptr %a, align 1
  %bv = load i64, ptr %b, align 1
  %r = call i64 @sub64(i64 %av, i64 %bv)
  store i64 %r, ptr %result, align 1
  ret void
}

; Wrapper: void ashr64_w(i8* result, i8* a, i16 count)
; Exercises __ashrdi3 libcall (i64 shifts route to libcall via
; customForCartesianProduct({s64}, {s8}) in the legalizer).
define dso_local void @ashr64_w(ptr %result, ptr %a, i16 %count) {
  %av = load i64, ptr %a, align 1
  %ext = zext i16 %count to i64
  %r = ashr i64 %av, %ext
  store i64 %r, ptr %result, align 1
  ret void
}
