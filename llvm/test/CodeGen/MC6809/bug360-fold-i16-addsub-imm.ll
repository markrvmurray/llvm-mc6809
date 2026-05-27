; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
;
; Bug #360: an i16 add/sub of an immediate is byte-decomposed by the
; legalizer (addb #lo; adca #hi) instead of using the native 16-bit ADDD.
; MC6809FoldAddSub16 refolds that byte carry-chain back into a single
; Add_i16_Imm (-> ADDD) on selected SSA MIR at -O1+. A `- C` reaches the
; backend as `+ (-C)`.

define i16 @dec1(i16 %x) {
; CHECK-LABEL: dec1:
; CHECK:       addd #-1
; CHECK-NOT:   adca
  %r = add i16 %x, -1
  ret i16 %r
}

define i16 @sub7(i16 %x) {
; CHECK-LABEL: sub7:
; CHECK:       addd #-7
; CHECK-NOT:   adca
  %r = sub i16 %x, 7
  ret i16 %r
}

; reg/reg form: the second operand spills to the stack, so the fold targets
; Add_i16_Mem / Sub_i16_Mem (ADDD/SUBD <mem>) instead of the addb/adca chain.
define i16 @add_rr(i16 %a, i16 %b) {
; CHECK-LABEL: add_rr:
; CHECK:       addd {{[0-9]+}},s
; CHECK-NOT:   adca
  %r = add i16 %a, %b
  ret i16 %r
}

define i16 @sub_rr(i16 %a, i16 %b) {
; CHECK-LABEL: sub_rr:
; CHECK:       subd {{[0-9]+}},s
; CHECK-NOT:   sbcb
  %r = sub i16 %a, %b
  ret i16 %r
}

; CC guard: usub-sat branches on the subtract's borrow ($c), so the chain's
; flags are consumed — the fold must be declined (otherwise the flag-set
; moves past the branch). Codegen must keep the byte-wise sbcb borrow, not a
; single SUBD whose $c the branch would read before it's computed.
declare i16 @llvm.usub.sat.i16(i16, i16)
define i16 @usat16(i16 %a, i16 %b) {
; CHECK-LABEL: usat16:
; CHECK-NOT:   subd
  %r = call i16 @llvm.usub.sat.i16(i16 %a, i16 %b)
  ret i16 %r
}

; Part 2 (compare elision): the loop test `addd #-1; cmpd #0; bne` collapses
; to `addd #-1; bne` — ADDD already set Z, and BNE (NE) tests only Z, so the
; CMPD #0 is redundant and dropped (MC6809LateOptimization).
define void @dec_loop(ptr %p, i16 %n) {
; CHECK-LABEL: dec_loop:
; CHECK:       addd #-1
; CHECK-NOT:   cmpd #0
; CHECK:       bne
entry:
  br label %loop
loop:
  %i = phi i16 [ %n, %entry ], [ %d, %loop ]
  store volatile i16 %i, ptr %p
  %d = add i16 %i, -1
  %c = icmp ne i16 %d, 0
  br i1 %c, label %loop, label %exit
exit:
  ret void
}

; TST elision: an ACC op (here ANDB) sets N/Z, so the following TSTB before a
; Z/N-only branch is redundant and dropped. The ACC op must define the tested
; register at its own width — a wider def (e.g. LDD before TSTB) is rejected,
; since LDD's flags reflect the 16-bit value, not the byte TSTB tests.
define void @tst_elide(i8 %a, i8 %b, ptr %p) {
; CHECK-LABEL: tst_elide:
; CHECK:       andb
; CHECK-NOT:   tstb
; CHECK:       lb{{eq|ne}}
entry:
  %x = and i8 %a, %b
  %c = icmp ne i8 %x, 0
  br i1 %c, label %nz, label %z
nz:
  store volatile i8 1, ptr %p
  br label %z
z:
  ret void
}

; Carry-propagation guard: a wider (i32) add must NOT be folded. Its low
; 16 bits' carry-out feeds the high half, and ADDD would not thread the
; imaginary carry register the next byte consumes — so no part of an i32
; add may collapse to ADDD. (The first cut of the pass got this wrong and
; miscompiled codegen-add32.s.)
define i32 @add32(i32 %a, i32 %b) {
; CHECK-LABEL: add32:
; CHECK-NOT:   addd
  %r = add i32 %a, %b
  ret i32 %r
}
