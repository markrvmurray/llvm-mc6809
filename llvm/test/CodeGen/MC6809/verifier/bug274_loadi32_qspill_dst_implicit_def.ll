; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #274 residue (closed 2026-05-11):
; expandLoadIdx's Bug #221 Q-spill DESTINATION path emits a two-LDD
; slot-to-slot copy to populate $spill_q*'s stack slot — but neither
; the LDDs nor the STDs annotate the implicit-def of $spill_q*. After
; the original Load_i32_Mem (whose operand 0 was the $spill_q* DEF)
; is erased, the verifier sees no machine-level def of $spill_q*
; through this expansion. Any downstream consumer that opaquely
; references the register — clang's FAKE_USE debug intrinsics being
; the typical surface — then trips "Using an undefined physical
; register".
;
; Fix: add `RegState::Implicit` def of the SPILL_Q* on the last STD
; of the two-LDD pair. The slot DOES hold the value after the copy;
; this just tells the verifier so.
;
; Closes 2 of 6 Og-hd6309-mame residual hits (FAKE_USE killed
; renamable $spill_q0 ×2 in strncat_s).

; CHECK-LABEL: spill_q_load_with_fake_use:
; CHECK-NOT: Bad machine code

declare void @sink_i32(i32, i32, i32, i32, i32)

define i32 @spill_q_load_with_fake_use(i32 %a, i32 %b, i32 %c, i32 %d) {
entry:
  ; Force i32 values to spill into SPILL_Q* slots, then call a
  ; multi-arg function so the spilled values must be reloaded.
  call void @sink_i32(i32 %a, i32 %b, i32 %c, i32 %d, i32 0)
  %r = add i32 %a, %d
  ret i32 %r
}
