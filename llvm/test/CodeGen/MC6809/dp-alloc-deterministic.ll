; RUN: llc -mtriple=mc6809 -dp-avail=4 -verify-machineinstrs < %s | FileCheck %s

; Direct-page allocation must be deterministic.
;
; Every one of these globals is stored exactly once from the same basic block,
; so MC6809DirectPageAlloc gives all eight the same benefit, and only four of
; them fit in the four direct-page bytes it is given.  The winners are decided
; entirely by the order the candidates were collected in.  That order must be
; the order of the program, not of pointers, or the same input could link
; differently from one run to the next.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

@g0 = global i8 undef, align 1
@g1 = global i8 undef, align 1
@g2 = global i8 undef, align 1
@g3 = global i8 undef, align 1
@g4 = global i8 undef, align 1
@g5 = global i8 undef, align 1
@g6 = global i8 undef, align 1
@g7 = global i8 undef, align 1

; The first four globals collected get the direct page and direct addressing;
; the rest keep extended addressing.

; CHECK-LABEL: main:
; CHECK:      stb <mc6809_8(g0)
; CHECK-NEXT: stb <mc6809_8(g1)
; CHECK-NEXT: stb <mc6809_8(g2)
; CHECK-NEXT: stb <mc6809_8(g3)
; CHECK-NEXT: stb g4
; CHECK-NEXT: stb g5
; CHECK-NEXT: stb g6
; CHECK-NEXT: stb g7
define void @main() {
entry:
  store volatile i8 0, ptr @g0
  store volatile i8 0, ptr @g1
  store volatile i8 0, ptr @g2
  store volatile i8 0, ptr @g3
  store volatile i8 0, ptr @g4
  store volatile i8 0, ptr @g5
  store volatile i8 0, ptr @g6
  store volatile i8 0, ptr @g7
  ret void
}
