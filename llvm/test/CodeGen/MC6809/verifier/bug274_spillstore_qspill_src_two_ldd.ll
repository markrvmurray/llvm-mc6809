; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #274 residue (closed 2026-05-11):
; expandStoreIdx's `needsMaterialization` path emergency-saves $ad to
; a stack slot before materialising a SPILL_Q* source into AQ via LDQ.
; The save is `STD $ad, emergency_slot, $su, implicit $ad` — but $ad
; isn't always live at the SpillStore_i32_Mem call site, and the
; emergency save then reads an undefined physical register. The
; verifier flags it.
;
; Fix mirrors Bug #221's expandLoadIdx Q-spill DEST path: when the
; source is a SPILL_Q*, do a slot-to-slot two-LDD copy (LDD src+H →
; AD; STD AD → dst+H for H in {0, 2}). Only AD is touched, and the
; LDD's implicit-def $ad satisfies the verifier without requiring a
; pre-existing $ad value.
;
; Closes 3 of 6 Og-hd6309-mame residual hits (STDi_o8 implicit $ad
; undef in __bufio_flush_locked, strncpy_s ×2).

; CHECK-LABEL: store_i32_through_spill:
; CHECK-NOT: Bad machine code

@dst = external global i32

declare void @sink(i32, i32, i32, i32, i32, i32)

define void @store_i32_through_spill(i32 %a, i32 %b, i32 %c) {
entry:
  ; Force at least one i32 to spill to SPILL_Q*, then store it to
  ; memory. The spill framework's SpillStore_i32_Mem expansion will
  ; hit the Q-spill SOURCE path.
  call void @sink(i32 %a, i32 %b, i32 %c, i32 %a, i32 %b, i32 %c)
  store volatile i32 %a, ptr @dst
  store volatile i32 %b, ptr @dst
  store volatile i32 %c, ptr @dst
  ret void
}
