; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Bug #271 category 4 sentinel: post-RA expansion of LSL_i8/i16_Reg,
; LSR_i8/i16_Reg, ASR_i8/i16_Reg, ROL_i8_Reg, and ROR_i8_Reg used to
; remove only operands 0 (dst) and 1 (src) from the MC6809ShiftBase
; pseudo, leaving operand 2 (the i8imm shift count) stranded as an
; explicit arg on the now-zero-arity ASL/LSR/ASR/ROL/RORa concrete
; instruction. -verify-machineinstrs flagged it as "Extra explicit
; operand on non-variadic instruction" (9 hits in libc/string/memmem.c
; alone at -Og hd6309).
;
; Fix: also remove operand 2 first so all three explicit operands are
; gone before the setDesc + addImplicitDefUseOperands sequence.
;
; Test uses shl/lshr (which become ASLBa/LSRBa). ashr (ASRBa) and
; the rotate forms also have the fix applied but the simple test
; patterns trip a separate pre-existing verifier issue ("Using an
; undefined physical register" on $c) — covered separately under
; cat-1 / cat-3 of bug #271.

define i8 @bug271_cat4_shl(i8 %x) {
; CHECK-LABEL: bug271_cat4_shl:
; CHECK: aslb
  %r = shl i8 %x, 1
  ret i8 %r
}

define i8 @bug271_cat4_lshr(i8 %x) {
; CHECK-LABEL: bug271_cat4_lshr:
; CHECK: lsrb
  %r = lshr i8 %x, 1
  ret i8 %r
}
