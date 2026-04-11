; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
;
; Bug #90 regression test: G_SEXT from s1 to s8 selects SEX8Implicit,
; which must be expanded in expandPostRAPseudo. Previously the switch
; had no case for SEX8Implicit, so the pseudo survived to the asm
; printer and llc crashed with "Pseudoinstruction was never lowered".
;
; The expansion emits:
;   [TFR src_byte, dst_byte]  if src and dst live in different halves
;   AND[AB] #1                 (mask off all but the LSB)
;   NEG[AB]                    (0 -> 0, 1 -> -1 = 0xFF)
;
; Fingerprint in the asm: `andb #1` followed by `negb` (or `anda`/`nega`).
; The TFR prefix is present when regalloc assigned source and dest to
; different halves of D.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: sext_s1_to_s8:
; CHECK:      and{{[ab]}} #1
; CHECK-NEXT: neg{{[ab]}}
define i8 @sext_s1_to_s8(i8 %x) {
  %is_zero = icmp eq i8 %x, 0
  %ext = sext i1 %is_zero to i8
  ret i8 %ext
}
