; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
;
; Bug #359: pointer compares stay in the INDEX domain (CMPX/CMPY) instead
; of being dragged into D.  When the compared pointer has been spilled
; (SPILL_X), MaterializeSpills must materialize it through IY, NOT IX —
; IX holds the incoming first pointer argument (%pwc) live across the
; second pointer's null-check.
;
; The bug: the spilled %s operand of `s == NULL` was left for
; expandCompareImm, which loaded it into IX for `cmpx #0` (getRealRegForSpill
; returns IX), silently destroying %pwc.  The later `store i16, ptr %pwc`
; then addressed through %s, so the value was written into the source
; object and *%pwc stayed unmodified.  This was the __ascii_mbtowc
; miscompile (test-uchar): the converted character landed in the input
; string and the output wchar stayed 0.
;
; Fix: the s-compare materializes through IY (`cmpy #0`), leaving IX = %pwc
; untouched for the store.

define i16 @two_ptr_store(ptr %pwc, ptr %s, i16 %n) {
; CHECK-LABEL: two_ptr_store:
; The spilled %s is loaded into Y — it must NOT be loaded into X (which still
; holds %pwc). The redundant `cmpy #0` for the null-check is elided by Bug
; #360 since the load already set Z.
; CHECK:       ldy
; %pwc is the incoming arg in X and stays there: its own null-check is a
; direct CMPX and the store addresses through X (not through %s in Y).
; CHECK:       cmpx #0
; CHECK:       std ,x
entry:
  %sn = icmp eq ptr %s, null
  br i1 %sn, label %ret0, label %c1
c1:
  %nn = icmp eq i16 %n, 0
  br i1 %nn, label %ret2, label %c2
c2:
  %ch = load i8, ptr %s
  %neg = icmp slt i8 %ch, 0
  br i1 %neg, label %ret3, label %store
store:
  %pn = icmp eq ptr %pwc, null
  br i1 %pn, label %cont, label %dostore
dostore:
  %z = zext i8 %ch to i16
  store i16 %z, ptr %pwc
  br label %cont
cont:
  ret i16 1
ret0:
  ret i16 0
ret2:
  ret i16 -2
ret3:
  ret i16 -1
}
