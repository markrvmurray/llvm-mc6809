; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s
;
; Bug #299 sentinel — Load_i32_Mem with dst=SPILL_Q*N expands (via
; Bug #221's two-LDD path) to a pair of LDD instructions that
; transiently write AD.  AD is the sub_hi_word of AQ, so a *different*
; vreg live in $aq across the pseudo gets its high half silently
; clobbered.
;
; Manifest (Phase B re-applied): picolibc divsi_signed_probe at
; Os-lto-hd6309-mame.  chk_div receives two i32 args (`a` and `b`);
; the second's Load_i32_Mem with dst=$spill_q0 (via Phase B widened
; AQc) clobbered $aq holding `a`, propagating wrong values into the
; __divsi3 call.
;
; Fix: Load_i32_Mem declares `AD` in its implicit Defs (mirrors the
; sibling SpillLoad_i32_Mem's Defs per Bug #290).  AD sub-reg-aliases
; AQ so the regalloc now sees the conflict at allocation time and
; chooses not to keep an i32 in $aq across an LDD-emitting pseudo.
;
; This sentinel is dormant pre-Phase-B (i32 G_LOAD narrows to two
; i16 G_LOADs, never emits Load_i32_Mem).  The CHECK-NOT pattern
; trivially passes when there is no LDQ in the lowered output.
; Post-Phase-B-re-land, the same CHECK-NOT asserts that no LDQ
; intermediate result is corrupted by an interleaved LDD — i.e.
; the regalloc must not emit the pattern `LDQ ; LDD ; STQ` where
; the LDD lands between the LDQ-produced AQ value and its STQ
; consumer.

define void @bug299_two_i32_args(i32 %a, i32 %b, ptr %out_a, ptr %out_b) {
; CHECK-LABEL: bug299_two_i32_args:
entry:
  store i32 %a, ptr %out_a, align 1
  store i32 %b, ptr %out_b, align 1
  ret void
}

; The bug pattern is: LDQ <a-slot>,u ; LDD <b-half>,u ; STQ ,<x>
; — the middle LDD silently overwrites AQ's high half before the
; STQ saves the (now corrupted) AQ.  Assert the lowered output
; never has an LDD instruction sitting between an LDQ and the
; first subsequent STQ.
;
; Realised as: no LDD-from-U-relative-slot occurs between the LDQ
; load of an i32 argument and the corresponding STQ store.  When
; Phase B is reverted (current default), no LDQ is emitted at all,
; and the CHECK-NOT is vacuously satisfied.

; CHECK-NOT: ldq{{[^,]*}},u{{[^;]*}};{{[^;]*}}ldd{{[^;]*}};{{[^;]*}}stq
