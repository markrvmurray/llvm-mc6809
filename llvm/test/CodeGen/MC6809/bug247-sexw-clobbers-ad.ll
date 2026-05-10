; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 < %s | FileCheck %s
;
; Bug #247 sentinel: HD6309 SEXW (sext i16 -> i32) physically clobbers AD,
; and the SEX32Implicit pseudo that lowers to it must declare AD in its
; Defs so regalloc spills any AD-resident value that is live across the
; pseudo (including the dst-in-SPILL_Q* case).
;
; Pre-fix: SEXWx had Uses=[AD] and SEX32Implicit's Defs omitted AD; an
; i16 value in AD that was needed after the sext (e.g. the `actual`
; operand of an i16 != i16 compare in test-timegm) was overwritten by
; sign(W), and the downstream comparison saw 0 instead of the real value.
; Symptom: 1009/1024 timegm iterations printed "got 0 want N" for h/m/s.
;
; The function below mirrors the shape: load two distinct i16 values,
; sign-extend one to i32 and use the result, then compare the second i16
; against a literal. If SEXW is allowed to clobber the second i16, the
; cmp-branch picks the wrong arm.

define void @bug247(ptr %p, ptr %q, ptr %sink) {
; CHECK-LABEL: bug247:
; CHECK: sexw
  %a = load i16, ptr %p
  %b = load i16, ptr %q
  %s = sext i16 %a to i32
  store i32 %s, ptr %sink
  %eq = icmp eq i16 %b, 12345
  br i1 %eq, label %t, label %f
t:
  store i16 %b, ptr %sink
  ret void
f:
  store i16 0, ptr %sink
  ret void
}
