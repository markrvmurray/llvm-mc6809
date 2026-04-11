; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
;
; Bug #92 regression test.
;
; The 8-bit `SubSetCarryUse_i8_Imm` (and symmetric AddSetCarryUse_i8_Imm)
; expansion used to skip emitting the sbca/adca instruction when the
; immediate was the identity value (0), on the assumption that `A - 0`
; is a no-op. But for the USE-variant the borrow-in (CC.C) is part of
; the computation: `A - 0 - borrow` is NOT the same as `A`. Dropping
; the instruction silently lost the borrow propagation from the low
; byte's subtract.
;
; Exposed by picolibc's `test_memcmp("Hello", "Help!", 5)` at -O1+:
; returned 0x00FC instead of the correct 0xFFFC. The IR is
; `sub i16 (zext i8), (zext i8)`, which byte-decomposes into a low-
; byte SUBB (sets borrow) and a high-byte SBCA of 0 from 0 (with
; borrow-in). Skipping the SBCA dropped the sign-extension and
; produced the positive value.
;
; Fix: use INT32_MIN as IdentityValue for the SubSetCarryUse /
; AddSetCarryUse expand paths so the skip never triggers. Same
; pattern as the pre-existing fix for SubSetCarry / AddSetCarry
; (bug #53).
;
; The asm must contain an `sbca` instruction for the high byte.

target triple = "mc6809-unknown-unknown"

; At -O0 the high-byte 0 operand lives in a stack slot (`sbca NN,u`).
; At -O1+ it's a constant immediate (`sbca #0`). Either must appear.
; CHECK-LABEL: sub_zext_i8:
; CHECK:      subb {{[0-9]+,u}}
; CHECK:      sbca {{#0|[0-9]+,u}}
define i16 @sub_zext_i8(i8 %a, i8 %b) {
entry:
  %conv_a = zext i8 %a to i16
  %conv_b = zext i8 %b to i16
  %sub = sub nsw i16 %conv_a, %conv_b
  ret i16 %sub
}

; Symmetric check for the add path: `add i16 (zext i8), (zext i8)`
; decomposes to ADDB + ADCA. With bug #92 the ADCA of zero was
; skipped, silently dropping the carry propagation.
; CHECK-LABEL: add_zext_i8:
; CHECK:      addb
; CHECK:      adca {{#0|[0-9]+,u}}
define i16 @add_zext_i8(i8 %a, i8 %b) {
entry:
  %conv_a = zext i8 %a to i16
  %conv_b = zext i8 %b to i16
  %sum = add i16 %conv_a, %conv_b
  ret i16 %sum
}
