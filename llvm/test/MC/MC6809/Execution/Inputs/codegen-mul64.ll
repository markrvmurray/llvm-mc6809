; LLVM IR functions for 64-bit multiply execution tests.
;
; gcc6809's long long is 4 bytes (non-conformant), so the C harness
; can't pass i64 directly — all operands and results travel through
; pointers via the _w wrappers.
;
; G_MUL for s64 routes to the __muldi3 libcall (no hand-asm in
; compiler-rt/lib/builtins/mc6809; the C version from compiler-rt
; gets pulled in via the toolchain). This test exercises that path.

target triple = "mc6809-unknown-unknown"

define internal i64 @mul64(i64 %a, i64 %b) {
  %r = mul i64 %a, %b
  ret i64 %r
}

define dso_local void @mul64_w(ptr %r, ptr %ap, ptr %bp) {
  %a = load i64, ptr %ap, align 1
  %b = load i64, ptr %bp, align 1
  %x = call i64 @mul64(i64 %a, i64 %b)
  store i64 %x, ptr %r, align 1
  ret void
}
