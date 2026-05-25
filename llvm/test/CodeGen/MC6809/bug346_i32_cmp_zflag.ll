; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #346: HD6309 i32 compares lower to SUBW #lo + SBCD #hi, after which
; CC.Z reflects ONLY the high 16 bits, not the full 32-bit difference.
;
;  - Signed GT/LE consume BOTH Z and V. The STQ Z-fix (which clears V)
;    can't be used, so they are rewritten to GE/LT against K+1, which read
;    only N/V (set correctly by SBCD) and need no Z-fix at all.
;  - EQ/NE/LS/HI read Z (not V), so they keep a STQ-to-scratch that sets a
;    correct full-32 Z before the branch.

; LE -> LT against K+1 (1000+1 = 1001); branch reads N/V only, so there
; must be NO stq Z-fix in this function.
; CHECK-LABEL: i32_sle_brcond:
; CHECK:      subw #1001
; CHECK-NEXT: sbcd #0
; CHECK:      {{l?}}b{{(ge|lt)}}
; CHECK-NOT:  stq
; CHECK-NOT:  __cmpsi2
define i16 @i32_sle_brcond(i32 %x) {
entry:
  %c = icmp sle i32 %x, 1000
  br i1 %c, label %t, label %f
t:
  ret i16 1
f:
  ret i16 2
}

; EQ needs a correct full-32 Z, so a STQ-to-scratch (leas -4 / stq / leas 4)
; must appear between the SUBW/SBCD and the branch.
; CHECK-LABEL: i32_eq_brcond:
; CHECK:      subw #22136
; CHECK:      sbcd #4660
; CHECK:      leas -4,s
; CHECK-NEXT: stq ,s
; CHECK-NEXT: leas 4,s
; CHECK:      {{l?}}b{{(eq|ne)}}
; CHECK-NOT:  __cmpsi2
define i16 @i32_eq_brcond(i32 %x) {
entry:
  %c = icmp eq i32 %x, 305419896
  br i1 %c, label %t, label %f
t:
  ret i16 1
f:
  ret i16 2
}
