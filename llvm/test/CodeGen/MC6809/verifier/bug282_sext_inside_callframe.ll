; RUN: llc -mtriple=mc6809 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #282 Shape B (closed 2026-05-12):
; `sext i32-call-result to i64` where the i32 source is the return
; value of a libcall whose call-frame slot is read via G_LOAD.
;
; Pre-fix, the combiner merged G_LOAD(s32) + G_SEXT(s64) into a
; single G_SEXTLOAD(s64 from s32-mem) placed at the G_LOAD's
; original position — which the IRTranslator emitted INSIDE the
; outer ADJCALLSTACK frame (the sret slot must be read before the
; frame is torn down). The G_SEXTLOAD's legalizeLoad then split it
; back into G_LOAD + G_SEXT at that inside-frame position. The
; upstream LegalizerHelper::lowerEXT then computed the sign byte
; via `G_ICMP slt(%src:s32, 0:s32)` — a wide-type ICMP that
; MC6809's legalizeCustom on G_ICMP routes to a `__cmpsi2`
; libcall, emitting an inner ADJCALLSTACK pair nested inside the
; outer. MachineVerifier rejected with
;   *** Bad machine code: FrameSetup is after another FrameSetup ***
;
; Replacing the wide ICMP with a narrower one (G_ICMP on the
; unmerged MSB byte) still hit a downstream verifier rule:
; G_ICMP s8 → i1 materialised via PHI of Load_i1_Imm 0/1 selects
; to a branch diamond, splitting the BB so the outer ADJCALLSTACK
; pair spans multiple BBs:
;   *** Bad machine code: Call frame size on entry does not match
;       value computed from predecessor ***
;
; Fix: custom G_SEXT (s64, s32) walks forward looking for the
; enclosing ADJCALLSTACKUP. If found AND no use of the SEXT's
; result lives between MI and the ADJCALLSTACKUP in the same MBB,
; override the MIRBuilder insertion point to AFTER the
; ADJCALLSTACKUP. The sign byte is then derived from the unmerged
; MSB byte via `0 - zext(slt(MSB, 0))`. The resulting branch
; diamond is OUTSIDE the outer call frame — harmless.
;
; Cross-MBB uses are naturally after FrameUp; same-MBB uses
; before FrameUp (e.g. the sext feeds the immediately-following
; libcall's argument stores) keep the default insertion point
; (BEFORE MI) — the diamond stays inside the frame and a
; subsequent libcall's CC clobber handles it.
;
; Closes the last 6 Og-fp verifier hits (test_lrintl,
; test_lrint_zero shapes).

; CHECK-LABEL: sext_i32_call_result_to_i64:
; CHECK-NOT: Bad machine code

declare i32 @some_libcall_returning_i32() nounwind

define i64 @sext_i32_call_result_to_i64() nounwind {
entry:
  %r = tail call i32 @some_libcall_returning_i32()
  %conv = sext i32 %r to i64
  ret i64 %conv
}
