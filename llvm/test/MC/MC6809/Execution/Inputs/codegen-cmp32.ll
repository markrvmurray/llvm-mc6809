; LLVM IR functions for 32-bit ICMP execution tests.
;
; Mirrors codegen-cmp64.ll one width down. gcc6809's int = i16 so
; we can't pass i32 by value across the language boundary; the
; harness uses pointer wrappers (_w suffix) instead.
;
; G_ICMP for s32 routes via __cmpsi2 / __ucmpsi2 (hand-asm in
; compiler-rt/lib/builtins/mc6809/) per the bug-#158 libcall-first
; strategy. This file exercises that path runtime-end-to-end.

target triple = "mc6809-unknown-unknown"

define internal i1 @eq32(i32 %a, i32 %b) {
  %c = icmp eq i32 %a, %b
  ret i1 %c
}

define internal i1 @ne32(i32 %a, i32 %b) {
  %c = icmp ne i32 %a, %b
  ret i1 %c
}

define internal i1 @slt32(i32 %a, i32 %b) {
  %c = icmp slt i32 %a, %b
  ret i1 %c
}

define internal i1 @sgt32(i32 %a, i32 %b) {
  %c = icmp sgt i32 %a, %b
  ret i1 %c
}

define internal i1 @ult32(i32 %a, i32 %b) {
  %c = icmp ult i32 %a, %b
  ret i1 %c
}

define internal i1 @ugt32(i32 %a, i32 %b) {
  %c = icmp ugt i32 %a, %b
  ret i1 %c
}

; Wrappers: return i1 result as i8 (0 or 1) through *r.

define dso_local void @eq32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @eq32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ne32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @ne32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @slt32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @slt32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @sgt32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @sgt32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ult32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @ult32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}

define dso_local void @ugt32_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i32, ptr %ap, align 1
  %b = load i32, ptr %bp, align 1
  %c = call i1 @ugt32(i32 %a, i32 %b)
  %x = zext i1 %c to i8
  store i8 %x, ptr %r, align 1
  ret void
}
