; RUN: llc -mtriple=mc6809 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -stop-after=instruction-select \
; RUN:   -o - %s 2>&1 | FileCheck %s

; Bug #302 redesign Phase 3 Stage 3.b (2026-05-17):
; The three Materialize* emission sites in MC6809InstructionSelector.cpp
; (cross-BB bridging at line ~336, G_ZEXT s8←s1 phantom-BIT1 at
; line ~787, G_ANYEXT s1→s8 phantom-BIT1 at line ~878) were switched
; from MaterializeCarryToByte_i8 / MaterializeOverflowToByte_i8 (which
; took a BIT1:$src vreg input) to MaterializeCC_C_to_byte /
; MaterializeCC_V_to_byte (no input — CC.C / CC.V is the sole
; authoritative source, declared via asymmetric `Defs = [NZ, V, C]`
; with NO Uses).
;
; The dropped BIT1 vreg input was a scheduling phantom (see the long
; comment block on MC6809ArithmeticBaseCarry in
; MC6809InstrFamilies.td:380-445 and Bug #186 v5).  Removing it
; breaks the BIT1 → sub_lsb → ACC32_with_sub_lsb intersection-class
; collapse that motivates Bug #302; Stage 3.c will follow with the
; structural drop of the BIT1 class and sub_lsb SubRegIndex.
;
; This test verifies the emission switch:
;   - carry-phantom anyext → MaterializeCC_C_to_byte (no BIT1 src)
;   - overflow-phantom anyext → MaterializeCC_V_to_byte (no BIT1 src)
; and that NEITHER MaterializeCarryToByte_i8 NOR
; MaterializeOverflowToByte_i8 (with their BIT1 src) survive.

; CHECK-LABEL: name: carry_anyext_to_byte
; A 16-bit addition with the carry-out anyext'd to i8 forces the
; G_ANYEXT s1→s8 phantom-BIT1 path.  Selector emits
; MaterializeCC_C_to_byte right after the producer; the byte result
; flows to the i8 return.
; CHECK:     MaterializeCC_C_to_byte
; CHECK-NOT: MaterializeCarryToByte_i8
; CHECK-NOT: MaterializeOverflowToByte_i8
define i8 @carry_anyext_to_byte(i16 %a, i16 %b) {
entry:
  %sum = call { i16, i1 } @llvm.uadd.with.overflow.i16(i16 %a, i16 %b)
  %carry = extractvalue { i16, i1 } %sum, 1
  %byte = zext i1 %carry to i8
  ret i8 %byte
}

; CHECK-LABEL: name: overflow_anyext_to_byte
; Signed-add overflow-out anyext'd to i8 takes the overflow-phantom
; path; MaterializeCC_V_to_byte should appear.
; CHECK:     MaterializeCC_V_to_byte
; CHECK-NOT: MaterializeOverflowToByte_i8
; CHECK-NOT: MaterializeCarryToByte_i8
define i8 @overflow_anyext_to_byte(i16 %a, i16 %b) {
entry:
  %sum = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %a, i16 %b)
  %ov = extractvalue { i16, i1 } %sum, 1
  %byte = zext i1 %ov to i8
  ret i8 %byte
}

declare { i16, i1 } @llvm.uadd.with.overflow.i16(i16, i16)
declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)
