; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 \
; RUN:     -stop-after=legalizer < %s | FileCheck %s
;
; G_SEXT (s16, s1) and (s32, s1) legalization regression test for bug #82a.
;
; clang lowers `(long)(int)(some_bool)` to a chained sext through G_SEXT
; intrinsics with a small source type and a much wider destination. The
; MC6809 selector has patterns only for the two single-step shapes the
; hardware supports natively:
;   (s8,  s1)  via SEX8Implicit
;   (s16, s8)  via SEX16Implicit (the native SEX instruction)
; Wider widenings have to be chained explicitly through both intermediate
; types — the selector does not synthesise multi-step extends by itself.
;
; The MC6809 G_SEXT rule custom-lowers (s16, s1) and (s32, s1) into the
; appropriate chains:
;
;   (s16, s1):  s1 → s8 → s16
;   (s32, s1):  s1 → s8 → s16 → s32
;                              ^^ this last step lowers via .lowerFor
;
; This test runs only as far as the legalizer (`-stop-after=legalizer`) so
; it documents the intent of the legalize pass without depending on the
; downstream regalloc, which is currently bug #82b — the s32 sext-from-s1
; expansion produces enough register pressure that the resulting MIR
; can't always allocate. The legalizer half is correct in isolation; the
; regalloc half is in the same problem class as bug #85b vfprintf.
;
; Each function below produces an internal i1 (compare result) and
; sign-extends it. We don't take an i1 as a function argument because
; the calling convention path for i1 has its own quirks unrelated to
; this bug.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: name: sext_s1_to_s16
; The legalizer must produce a chain G_SEXT %x_s1 -> s8, then -> s16.
; CHECK: G_SEXT
; CHECK: G_SEXT
define i16 @sext_s1_to_s16(i16 %x) {
  %is_zero = icmp eq i16 %x, 0
  %ext = sext i1 %is_zero to i16
  ret i16 %ext
}

; CHECK-LABEL: name: sext_s1_to_s32
; The legalizer must produce a chain through s8, s16, then s32 (the s32
; step also lowers via the existing .lowerFor({{s32, s16}}) rule).
; CHECK: G_SEXT
; CHECK: G_SEXT
; CHECK: G_SEXT
define i32 @sext_s1_to_s32(i16 %x) {
  %is_zero = icmp eq i16 %x, 0
  %ext = sext i1 %is_zero to i32
  ret i32 %ext
}
