; LLVM IR functions for 64-bit ICMP execution tests.
;
; gcc6809's long long is 4 bytes (non-conformant), so the C harness
; can't pass i64 directly. Instead, the harness calls pointer-based
; wrappers (_w suffix) that load operands through pointers and store
; the boolean result as a single i8 (0 or 1) for the harness to print.
;
; G_ICMP for s64 is `Custom` in MC6809LegalizerInfo and decomposes
; into __cmpdi2 / __ucmpdi2 libcalls (hand-written .S in
; compiler-rt/lib/builtins/mc6809). This file exercises that path.

target triple = "mc6809-unknown-unknown"

define internal i1 @eq64(i64 %a, i64 %b) {
  %c = icmp eq i64 %a, %b
  ret i1 %c
}

define internal i1 @ne64(i64 %a, i64 %b) {
  %c = icmp ne i64 %a, %b
  ret i1 %c
}

define internal i1 @slt64(i64 %a, i64 %b) {
  %c = icmp slt i64 %a, %b
  ret i1 %c
}

define internal i1 @sgt64(i64 %a, i64 %b) {
  %c = icmp sgt i64 %a, %b
  ret i1 %c
}

define internal i1 @ult64(i64 %a, i64 %b) {
  %c = icmp ult i64 %a, %b
  ret i1 %c
}

define internal i1 @ugt64(i64 %a, i64 %b) {
  %c = icmp ugt i64 %a, %b
  ret i1 %c
}

; Wrappers: return i1 result as i8 (0 or 1) through *r.

define dso_local void @eq64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @eq64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ne64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @ne64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @slt64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @slt64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @sgt64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @sgt64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ult64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @ult64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ugt64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %c = call i1 @ugt64(i64 %a, i64 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}
