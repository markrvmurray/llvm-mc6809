; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #275 (closed 2026-05-11):
; MC6809MaterializeSpills's backward LivePhysRegs analysis flagged
; NeedSaveD whenever any super-register of $ad (notably $aq) was live
; at the save point — because LivePhysRegs::addReg(super) adds all
; sub-regs into the live set. The forward pass then emitted
; `Store_i16_Mem $ad → emergency_slot` to preserve $ad's bits as part
; of the super-reg's value, even when $ad wasn't a direct MBB
; live-in. -verify-machineinstrs flagged the save's $ad operand as
; "Using an undefined physical register".
;
; The save IS semantically correct (it preserves $ad's bits across
; the upcoming LDD clobber so $aq's whole-32-bit value survives), but
; the operand's use was unannotated. Fix: when the save is at
; MBB.begin() and $ad isn't a direct live-in (nor any of its
; sub-regs), emit the save's source with `RegState::Undef`. STD still
; round-trips the bits; the verifier accepts the undef-marked use.
;
; Closes the last Og-hd6309-mame residual verifier hit in
; __xdrrec_getrec.

; CHECK-LABEL: spill_save_then_clobber_via_supreg:
; CHECK-NOT: Bad machine code

declare i32 @abi_i32_call(i32, i32)

define i32 @spill_save_then_clobber_via_supreg(i32 %a, i32 %b, i32 %c) {
entry:
  ; Force i32 traffic plus a side-effecting call to materialise the
  ; SpillStore + Bug #221 two-LDD expansion pattern that surfaces the
  ; super-reg-aliasing save-AD case.
  %x = call i32 @abi_i32_call(i32 %a, i32 %b)
  %y = call i32 @abi_i32_call(i32 %x, i32 %c)
  %z = add i32 %y, %a
  ret i32 %z
}
