; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
;
; Regression sentinel for bug #214 / bug #217 — byte-pair lowering of
; `sub i16 0, %iv` paired with `add i16 %base, %iv`.
;
; History: bug #209's Os-lto leg was originally closed via a
; `-disable-lsr` workaround (commit d357062e38a9, since retired). The
; first-pass fix (Contract B with `Defs += AA, AB` on byte _Reg
; pseudos) closed the test-getopt failure but blew regalloc pressure
; catastrophically at non-LTO opt levels (bug #217).
;
; The lasting fix: per-half routing.
;
;   * Byte _Reg pseudos' dst is constrained to ACC8_Bonly (AB +
;     SPILL_B*) — regalloc commits to B-half pre-RA.
;   * Their TableGen Defs declares only `AB` clobbered (not AA).
;     A-half is preserved without regalloc-inserted save/restore.
;   * path-(c) of emit6809RegByteFromMem emits `op ,s+`
;     (post-increment fold) at expansion time, retiring the historic
;     `op 0,s; LEAS 1,s` triple.
;
; This sentinel proves Phase 2 of the fix: path-(c) emits `,s+`
; directly — no trailing `LEAS 1,s` after a byte-arithmetic
; stack-relative op.

define i16 @bug214_byte_pair_inc1(i16 %a, i16 %b) {
entry:
  ; A 16-bit add forces the byte-pair carry chain: ADDB lo + ADCA hi.
  ; The byte SetCarry/SetCarryUse pseudos route through ACC8_Bonly so
  ; the LHS materializes into AB, AccReg = AB.
  %sum = add i16 %a, %b
  ret i16 %sum
}

; CHECK-LABEL: bug214_byte_pair_inc1:
; CHECK-NOT: leas 1,s
