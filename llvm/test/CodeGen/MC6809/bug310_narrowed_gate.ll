; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; Bug #310 follow-up sentinel — narrowed customIf gate.
;
; The initial Bug #310 fix gated `legalizeLoad`'s extended path on
; `MemSize < TypeSize`, which routed several other type-mismatch
; shapes through the same custom code.  That broader scope produced
; verifier hits on shapes that legalised correctly via the default
; path before the gate.
;
; Narrowed to: `TypeSize == 32 && MemSize == 16` only — the exact
; vfprintf_s shape that triggered the original cannot-select error.
;
; This sentinel exercises BOTH the still-handled s32-from-s16 shape
; (covered above by `bug310_g_load_partial_width.ll`) AND adjacent
; non-affected shapes that should now go through the default
; legalizer path without verifier hits.
;
; Compilation must succeed for all functions.

; Adjacent shape #1: i32 load from i32 memory (no partial-width gap).
define i32 @bug310_narrowed_i32_from_i32(ptr %p) {
; CHECK-LABEL: bug310_narrowed_i32_from_i32:
entry:
  %v = load i32, ptr %p, align 1
  ret i32 %v
}

; Adjacent shape #2: i16 load from i16 memory (i16 is the native
; pointer-sized value on MC6809).
define i16 @bug310_narrowed_i16_from_i16(ptr %p) {
; CHECK-LABEL: bug310_narrowed_i16_from_i16:
entry:
  %v = load i16, ptr %p, align 1
  ret i16 %v
}

; Adjacent shape #3: i16 load from i8 memory (zext) — partial-width
; but s16/s8, not the s32/s16 shape the narrowed gate targets.
define i16 @bug310_narrowed_i16_from_i8(ptr %p) {
; CHECK-LABEL: bug310_narrowed_i16_from_i8:
entry:
  %s = load i8, ptr %p, align 1
  %z = zext i8 %s to i16
  ret i16 %z
}

; CHECK: rts
