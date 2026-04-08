; Calling-convention validation: function-under-test side.
;
; Compiled by llc and linked against a gcc6809-compiled C harness
; (harness-cc.c). Both compilers target MC6809 ABI version 1, where
; arg dispatch is by *type* with two independent register slots:
;
;   - the first i8  arg fills the B-slot once
;   - the first i16 arg fills the X-slot once
;   - any further i8 (B already used) → 1-byte stack slot
;   - any further i16 (X already used) → 2-byte stack slot
;   - return i8  in B, return i16 in X
;
; gcc6809 emits the stack args via `stb ,-s` (i8) or `stx ,--s` (i16),
; pushed right-to-left, so on entry the first stack-passed arg sits
; just past the saved return address.
;
; The whole point of this test is to exercise the i8 stack-arg path
; (1-byte packed slots, fixed by bug #69). For test_add3(i8,i8,i8),
; the first i8 takes B and the other two go on the stack at 2,s and
; 3,s. Using i16 args anywhere would silently bypass that path. The
; CHECK lines in calling-convention.s pin down the expected output.

target triple = "mc6809-unknown-unknown"

define dso_local i8 @test_identity8(i8 %a) local_unnamed_addr {
  ret i8 %a
}

define dso_local i8 @test_add8(i8 %a, i8 %b) local_unnamed_addr {
  %r = add i8 %a, %b
  ret i8 %r
}

define dso_local i8 @test_add3(i8 %a, i8 %b, i8 %c) local_unnamed_addr {
  %s1 = add i8 %a, %b
  %r = add i8 %s1, %c
  ret i8 %r
}

; Nested call exercises arg-passing across a recursive boundary:
; nested holds %x in B on entry, must push 1 onto the stack as arg2,
; then call test_add8 (which reads arg1 from B, arg2 from 2,s).
define dso_local i8 @test_nested(i8 %x) local_unnamed_addr {
  %r = call i8 @test_add8(i8 %x, i8 1)
  ret i8 %r
}

; factorial(n) — recursive. For the values we test (n ≤ 5) the
; result fits in i8, so we use i8 multiply (the 6809 MUL instruction
; or the __mulqi3 libcall, depending on what the legalizer picks).
define dso_local i8 @test_factorial(i8 %n) local_unnamed_addr {
entry:
  %is_base = icmp ule i8 %n, 1
  br i1 %is_base, label %base, label %recurse
base:
  ret i8 1
recurse:
  %nm1 = sub i8 %n, 1
  %sub = call i8 @test_factorial(i8 %nm1)
  %prod = mul i8 %n, %sub
  ret i8 %prod
}
