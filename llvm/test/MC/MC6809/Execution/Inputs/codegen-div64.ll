; LLVM IR functions for 64-bit divide / modulo execution tests.
;
; Routes to __divdi3 / __udivdi3 / __moddi3 / __umoddi3 (in
; test/MC/MC6809/Execution/Inputs/divdi.inc — bug-#100 hand-asm
; replacing the original libcall, all four with shared udivmoddi4
; core).

target triple = "mc6809-unknown-unknown"

define internal i64 @sdiv64(i64 %a, i64 %b) {
  %r = sdiv i64 %a, %b
  ret i64 %r
}

define internal i64 @udiv64(i64 %a, i64 %b) {
  %r = udiv i64 %a, %b
  ret i64 %r
}

define internal i64 @srem64(i64 %a, i64 %b) {
  %r = srem i64 %a, %b
  ret i64 %r
}

define internal i64 @urem64(i64 %a, i64 %b) {
  %r = urem i64 %a, %b
  ret i64 %r
}

define dso_local void @sdiv64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %x = call i64 @sdiv64(i64 %a, i64 %b)
  store i64 %x, ptr %r, align 1
  ret void
}

define dso_local void @udiv64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %x = call i64 @udiv64(i64 %a, i64 %b)
  store i64 %x, ptr %r, align 1
  ret void
}

define dso_local void @srem64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %x = call i64 @srem64(i64 %a, i64 %b)
  store i64 %x, ptr %r, align 1
  ret void
}

define dso_local void @urem64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %x = call i64 @urem64(i64 %a, i64 %b)
  store i64 %x, ptr %r, align 1
  ret void
}
