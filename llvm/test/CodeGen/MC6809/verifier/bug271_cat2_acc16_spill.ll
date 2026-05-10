; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-2 sentinel.
;
; The spill framework's `loadStoreRegStackSlot` ACC16 path used to
; create a fresh ACC16-class temporary vreg and copy the original
; vreg into it before emitting `Store_i16_Mem` (and the symmetric
; load shape). The historical comment claimed "this code introduces
; subregisters", but Store_i16_Mem / Load_i16_Mem don't use sub-reg
; indices — they store/load the full 16-bit value. The fresh-vreg
; pattern was effectively dead bookkeeping that confused greedy's
; spill machinery: greedy considered the spill to "belong to" the
; original vreg and recorded the live segment as ending at the
; Store, but the Store's operand was the fresh vreg — leaving the
; verifier to flag "Instruction ending live segment doesn't read
; the register" on every 16-bit spill (96 such hits at -Og hd6309
; mame across libc.a pre-fix).
;
; Fix: pass the original Reg directly to Store_i16_Mem /
; Load_i16_Mem. Reg's class is already a subclass of ACC16 (the
; operand class), so the verifier accepts the operand. The COPY +
; fresh vreg are no longer emitted, so greedy's spill bookkeeping
; matches the actual MI shape.
;
; This sentinel forces multiple i16 values live across a multi-arg
; sink call, triggering 16-bit spills that exercise the simplified
; ACC16 path under `-verify-machineinstrs`.

target triple = "mc6809-unknown-unknown"

declare i16 @get_i16()
declare void @sink(i16, i16, i16, i16, i16, i16)

define i16 @bug271_cat2_acc16_spill(i16 %k) nounwind {
entry:
  %a = call i16 @get_i16()
  %b = call i16 @get_i16()
  %c = call i16 @get_i16()
  %d = call i16 @get_i16()
  ; sink takes more args than fit in registers — forces stack spills
  ; of the live i16s so the spill framework's ACC16 path is exercised.
  call void @sink(i16 %a, i16 %b, i16 %c, i16 %d, i16 %k, i16 %k)
  %r = add i16 %a, %b
  %r2 = add i16 %r, %c
  %r3 = add i16 %r2, %d
  ret i16 %r3
}
