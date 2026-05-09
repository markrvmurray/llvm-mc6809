; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O1 < %s | FileCheck %s
;
; Bug #243 Class B: regalloc/legalize residuals at -Og-hd6309-mame after
; #239 + #243 Class A.
;
;   1. G_TRUNC s32 -> s1 (hash_bigkey/__big_split): legalizer custom path
;      now chains s32 -> s16 -> s1 (the (s1, s16) handler already existed).
;   2. G_FREEZE on i32/i64 was keeping wide vregs alive across function
;      bodies, exhausting ACC32 (5 slots) at -Og where less inlining
;      preserves more virtual locals. Clamp G_FREEZE to s16 (same
;      rationale as #239's G_PHI clamp).
;
; This sentinel exercises both: a freeze on an i64 arg followed by a
; trunc-to-i1 of an i32 derived from it. Both must legalize and reach
; selection without "ran out of registers" or "unable to legalize".
;
; CHECK-LABEL: bug243_class_b:

define i16 @bug243_class_b(i64 %i) {
entry:
  %f = freeze i64 %i
  %lo = trunc i64 %f to i32
  %t  = trunc i32 %lo to i1
  %r  = zext i1 %t to i16
  ret i16 %r
}
