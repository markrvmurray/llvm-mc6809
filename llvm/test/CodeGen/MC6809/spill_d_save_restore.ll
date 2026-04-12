; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
;
; Bug #94 verification: 16-bit SPILL_D save/restore correctness.
;
; Exercises the NeedSaveD path in MaterializeSpills for a 16-bit
; spill-operand pseudo. The function has enough live i16 values
; to force the greedy regalloc to place at least one in a SPILL_D*
; register. The D save/restore must correctly preserve the non-spill
; live value in $ad across the materialization while the spill
; result goes to its slot via the dematerialize store-back.
;
; CHECK-LABEL: many_i16_ops:
; Must not crash, and must produce a return (proof the pipeline
; completes without assertion failures from MaterializeSpills or
; the asm printer).
; CHECK: rts

target triple = "mc6809-unknown-unknown"

declare i16 @use_i16(i16)

define i16 @many_i16_ops(i16 %a, i16 %b, i16 %c) {
entry:
  %x = add i16 %a, %b
  %y = sub i16 %c, %a
  %z = add i16 %x, %y
  %w = call i16 @use_i16(i16 %z)
  %r = add i16 %w, %x
  %s = sub i16 %r, %y
  ret i16 %s
}
