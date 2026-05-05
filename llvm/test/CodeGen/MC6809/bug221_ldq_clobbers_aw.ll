; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 %s -o - | FileCheck %s
;
; Regression sentinel for bug #221 — LDQ clobbers AW between an
; earlier i16 reload and a later store of the reloaded value.
;
; Symptom (closed at picolibc test-fmemopen, test19_body line 982,
; Os-hd6309-mame): a function call returning i32 (e.g. `ftello`) had
; its low half spilled to a scratch slot and its high half (= 0 from
; zero-extension) left in physical AW, with a downstream
; (off_t)i + 1 i32 zext spilled to an imaginary `$spill_q*` reg.
; Post-RA materialization of `$spill_q*` emitted `LDQ` to load 4
; bytes into physical AQ — but AQ's low half IS AW. The LDQ silently
; clobbered the live AW that regalloc had committed independently
; (because $spill_q* is an imaginary reg distinct from physical AQ
; at regalloc time, no conflict was flagged). Concretely at the bug
; test's iteration i=1: `__ucmpsi2` saw LHS = 0x00010002 instead of
; 0x00000002 (the upper 16 bits carried `i` — what LDQ left in AW —
; not the i32 zero-extension's expected zero), and the comparison
; went `LHS > RHS` instead of `LHS == RHS`. Assert fired.
;
; Fix: in `expandLoadIdx`, when the destination of `Load_i32_Mem` is
; a Q-spill register, copy the 4 bytes via 2x `LDD`/`STD` (16-bit
; halves) instead of staging through physical AQ via `LDQ`/`STQ`.
; This touches only AD; AW and the rest of AQ stay untouched,
; preserving any live value regalloc placed in those sub-registers.
;
; This sentinel proves: an i32 load-from-spill into a wider
; computation no longer emits `LDQ`. Greppable. The exact form is a
; 4-byte load from a frame slot followed by 4-byte storage to a
; downstream slot.

@gptr = external global ptr

define i32 @bug221_load_i32_from_spill(i32 %x) #0 {
entry:
  %fp = load ptr, ptr @gptr, align 1
  %a = call i32 @sink_i32(ptr %fp)         ; produces an i32 callee result
  %z = zext i16 0 to i32                   ; force a separate i32 alongside
  %r = add i32 %a, %z                      ; uses the i32 path
  %s = add i32 %r, %x
  ret i32 %s
}

declare i32 @sink_i32(ptr)

attributes #0 = { nounwind }

; CHECK-LABEL: bug221_load_i32_from_spill:
; The fix replaces LDQ <slot>,u with two LDD halves. Assert that no
; LDQ from a frame slot appears in the lowered output. (LDQ on
; immediate constants — `ldq #N` — is fine; only the LDQ-from-slot
; pattern is the bug.)
; CHECK-NOT: ldq	{{.*}},u
