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
; Bug #311 Phase 2 update (2026-05-21): native HD6309 i32 carry chains
; mean i64 abs now lowers to two i32 sub-with-carry pseudos in one BB
; (no cross-BB bridge needed).  The original MaterializeCC_*_to_byte
; trigger via i64 abs is therefore obsolete here; this test now
; verifies that the i64-abs codegen takes the native i32 chain (proof
; that Phase 2 wiring is active) and that the carry-in operand on the
; AddSetCarryUse_i32_* pseudo is the same implicit-vreg shape the
; Bug #307 fix put in place for cross-BB MaterializeCC bridges.

; The AddSetCarryUse_i32_Imm consumer's phantom_carry input must be
; IMPLICIT (`implicit %<vreg>`), matching the AddSetCarry_i32_Imm
; producer's implicit-def.  Pre-Bug-#307-fix: explicit operand would
; trip the verifier with "Extra explicit operand on non-variadic
; instruction" once the i32 chain reaches post-RA.
; CHECK: AddSetCarry_i32_Imm
; CHECK-SAME: implicit-def %{{[0-9]+}}
; CHECK: AddSetCarryUse_i32_Imm
; CHECK-SAME: implicit %{{[0-9]+}}

define i64 @imaxabs_i64(i64 %j) {
entry:
  %neg = sub i64 0, %j
  %cmp = icmp slt i64 %j, 0
  %res = select i1 %cmp, i64 %neg, i64 %j
  ret i64 %res
}
