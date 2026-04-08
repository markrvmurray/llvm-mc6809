; Calling-convention validation: 16-bit function-under-test side.
;
; Compiled by llc and linked against gcc6809-compiled harness-cc16.c.
; Both compilers target MC6809 ABI version 1: arg dispatch is by
; *type* with two independent register slots — the first i8 arg
; fills B, the first i16 arg fills X, anything past those goes on
; the stack (i8 in 1-byte slots, i16 in 2-byte slots). Return i16
; in X.
;
; The functions here take only i16 args, so for f(i16, i16) the
; first arg fills X and the second goes on the stack. gcc6809
; pushes it via `stx ,--s` so it sits at S+2 after jsr. LLVM-MC6809
; reads it back with `ldd 2,s` (D is just a transit register loading
; the 2-byte slot, not part of the arg-passing convention) and
; combines with X via `leax d,x`. The CHECK lines in
; calling-convention-16.s pin down the expected hex output.

target triple = "mc6809-unknown-unknown"

define dso_local i16 @test_identity16(i16 %a) local_unnamed_addr {
  ret i16 %a
}

define dso_local i16 @test_add16(i16 %a, i16 %b) local_unnamed_addr {
  %r = add i16 %a, %b
  ret i16 %r
}

define dso_local i16 @test_sub16(i16 %a, i16 %b) local_unnamed_addr {
  %r = sub i16 %a, %b
  ret i16 %r
}
