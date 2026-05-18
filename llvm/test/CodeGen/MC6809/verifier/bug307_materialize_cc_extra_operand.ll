; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -stop-after=finalize-isel %s -o - | FileCheck %s

; Bug #307 (2026-05-18): MaterializeCC_C_to_byte / MaterializeCC_V_to_byte
; are the Bug #302 Stage 3.a pseudos with `InOperandList = (ins)` -- no
; explicit inputs.  CC.C/V is read directly via Uses=[C]/Uses=[V].
;
; The InstructionSelector's `ensureCarryChainIntegrity` bridge previously
; emitted `.addUse(CarryIn)` as an EXPLICIT operand on the producer-side
; ToBytePseudo, which the post-RA verifier rejected with
; "Extra explicit operand on non-variadic instruction".
;
; Fix: emit the use as IMPLICIT (`RegState::Implicit`).  Implicit
; operands keep CarryIn alive across the bridge for regalloc
; bookkeeping but don't count against the pseudo's explicit operand
; list -- verifier accepts.
;
; The i64-abs shape in imaxabs/llabs triggers the bridge insertion via
; the SUBE chain's borrow flag.  Stop after isel so we can inspect the
; bridge MI directly, before downstream passes (which would crash on
; the unrelated #308 / phantom_carry-COPY shapes at -O1).

; The bridge MI must have the CarryIn use as IMPLICIT, not EXPLICIT.
; Pre-fix: `MaterializeCC_C_to_byte %62:phantom_carry, ...` (explicit).
; Post-fix: `MaterializeCC_C_to_byte ..., implicit %62` (implicit).
; CHECK: MaterializeCC_{{[CV]}}_to_byte
; CHECK-SAME: implicit %{{[0-9]+}}

define i64 @imaxabs_i64(i64 %j) {
entry:
  %neg = sub i64 0, %j
  %cmp = icmp slt i64 %j, 0
  %res = select i1 %cmp, i64 %neg, i64 %j
  ret i64 %res
}
