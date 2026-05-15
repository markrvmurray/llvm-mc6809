; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s
;
; Bug #300 sentinel (symmetric to Bug #299) — Store_i32_Mem with
; src=SPILL_Q*N expands (mirror of Bug #221's two-LDD-slot-to-slot
; pattern on the store side) to a pair of LDD + STD instructions
; that transiently write AD.  AD is the sub_hi_word of AQ, so a
; *different* vreg live in $aq across the pseudo gets its high
; half silently clobbered.
;
; Manifest (Phase B re-applied): picolibc sqrt_probe, test-mc6809-fp
; at Os-lto-hd6309-mame.  These tests pass i64 (f64) args to
; libcalls, where the i64 narrowing produces Store_i32_Mem ops
; with src=SPILL_Q*N alongside live AQ vregs.
;
; Fix: Store_i32_Mem declares `AD` in implicit Defs (mirrors the
; sibling SpillStore_i32_Mem's Defs per Bug #290).  AD sub-reg-
; aliases AQ so the regalloc now sees the conflict at allocation
; time and chooses not to keep an i32 in $aq across a SPILL_Q*N
; Store_i32_Mem.
;
; Dormant pre-Phase-B (no Store_i32_Mem emitted; i32 stores narrow
; to two i16 STDs).  CHECK-NOT is vacuously satisfied when there
; are no STQ-from-AQ-with-LDD-interleaved sequences.

define void @bug300_two_i32_stores(i32 %a, i32 %b, ptr %out_a, ptr %out_b) {
; CHECK-LABEL: bug300_two_i32_stores:
entry:
  store i32 %a, ptr %out_a, align 1
  store i32 %b, ptr %out_b, align 1
  ret void
}

; Mirror of bug299_load_i32_aq_clobber.ll: the bug pattern is an
; LDD between an STQ-of-AQ and a subsequent STQ-of-AQ where the
; first STQ's AQ value gets corrupted before the second STQ runs.
; CHECK-NOT: stq{{[^,]*}},{{[^;]*}};{{[^;]*}}ldd{{[^;]*}};{{[^;]*}}stq
