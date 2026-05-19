; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #310 sentinel — G_LOAD with type-size > MMO-size (partial-width
; load) must be legalised.
;
; Symptom: at -O2-hd6309, vfprintf_s lowering produced a G_LOAD whose
; destination type was i32 (acc32) but whose MachineMemOperand was
; size=2 bytes.  The InstructionSelector rejected with
;   "cannot select: %12:acc32 = G_LOAD %11(p0) :: (load (s16) ...)"
;
; Fix: `MC6809LegalizerInfo` added a `customIf` for G_LOAD where the
; destination type-size exceeds the MMO-size; `legalizeLoad` splits
; into G_LOAD of the MMO-size followed by a G_ZEXT to the destination
; type.
;
; Compilation must succeed (-global-isel-abort=1 + -verify-machineinstrs).

define i32 @bug310_load_zext_i16_to_i32(ptr %p) {
; CHECK-LABEL: bug310_load_zext_i16_to_i32:
entry:
  %s = load i16, ptr %p, align 1
  %z = zext i16 %s to i32
  ret i32 %z
}

; CHECK: rts

define i32 @bug310_load_sext_i16_to_i32(ptr %p) {
; CHECK-LABEL: bug310_load_sext_i16_to_i32:
entry:
  %s = load i16, ptr %p, align 1
  %z = sext i16 %s to i32
  ret i32 %z
}

; CHECK: rts
