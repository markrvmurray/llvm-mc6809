; Bug #265 / Bug #296 Phase 2 sentinel: i32 register-pressure capacity.
;
; This test holds 8 i32 values simultaneously live, well above the old
; ACC32 capacity of 5 elements (AQ + SPILL_Q0..3) that Bug #265 surfaced.
; Before Bug #296's bump from SPILL_Q0..3 to SPILL_Q0..31, this pattern
; would have failed at register allocation with "ran out of registers
; during register allocation in function 'test_8i32_live_vregs'" at
; -O2 -mcpu=hd6309.  After the bump, ACC32 has 33 elements (AQ + 32
; spill slots) and the same pattern compiles cleanly.
;
; The shape mirrors what optimizer-generated word-loop patterns produce
; in real code (memccpy / rawmemchr at HD6309 -O2 — see Bug #265 body)
; — multiple i32 values held live across xor/add chains.

; RUN: llc -mtriple=mc6809-unknown-unknown -mcpu=hd6309 -O2 \
; RUN:     -global-isel -global-isel-abort=1 \
; RUN:     -verify-machineinstrs < %s | FileCheck %s

target datalayout = "e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Pf64:8-S8"
target triple = "mc6809-unknown-unknown"

define i32 @test_8i32_live_vregs(ptr %p, i32 %mask) {
entry:
  %p0 = getelementptr inbounds i32, ptr %p, i16 0
  %p1 = getelementptr inbounds i32, ptr %p, i16 1
  %p2 = getelementptr inbounds i32, ptr %p, i16 2
  %p3 = getelementptr inbounds i32, ptr %p, i16 3
  %p4 = getelementptr inbounds i32, ptr %p, i16 4
  %p5 = getelementptr inbounds i32, ptr %p, i16 5
  %p6 = getelementptr inbounds i32, ptr %p, i16 6
  %p7 = getelementptr inbounds i32, ptr %p, i16 7

  ; Load eight i32 values.  All are simultaneously live until the
  ; final reduction, so the legalizer / regalloc need to be able to
  ; juggle 8 i32 vregs at once.
  %v0 = load i32, ptr %p0, align 1
  %v1 = load i32, ptr %p1, align 1
  %v2 = load i32, ptr %p2, align 1
  %v3 = load i32, ptr %p3, align 1
  %v4 = load i32, ptr %p4, align 1
  %v5 = load i32, ptr %p5, align 1
  %v6 = load i32, ptr %p6, align 1
  %v7 = load i32, ptr %p7, align 1

  ; XOR each with the mask.
  %x0 = xor i32 %v0, %mask
  %x1 = xor i32 %v1, %mask
  %x2 = xor i32 %v2, %mask
  %x3 = xor i32 %v3, %mask
  %x4 = xor i32 %v4, %mask
  %x5 = xor i32 %v5, %mask
  %x6 = xor i32 %v6, %mask
  %x7 = xor i32 %v7, %mask

  ; Reduce.  Each %s* keeps several of the %x* values live until it's
  ; used.
  %s01 = add i32 %x0, %x1
  %s23 = add i32 %x2, %x3
  %s45 = add i32 %x4, %x5
  %s67 = add i32 %x6, %x7
  %s0123 = add i32 %s01, %s23
  %s4567 = add i32 %s45, %s67
  %sum = add i32 %s0123, %s4567

  ret i32 %sum
}

; Just confirm the function emitted at all — the meaningful assertion
; is that llc didn't bail with "ran out of registers".  We don't pin a
; specific instruction sequence because regalloc decisions are sensitive
; to small backend changes and would make this test flaky.
;
; CHECK-LABEL: test_8i32_live_vregs:
; CHECK:       rts
