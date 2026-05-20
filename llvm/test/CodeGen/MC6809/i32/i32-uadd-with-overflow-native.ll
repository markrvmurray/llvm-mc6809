; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=HD6309
; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=MC6809
;
; Bug #311 Phase 2: i32 add-with-overflow intrinsic on HD6309 stays at
; i32 width through to native ADDW+ADCD codegen.  Pre-Phase-2 the
; legalizer narrowed s32 G_UADDO/G_UADDE to s8, producing four byte
; adds in series.  Post-Phase-2 the s32 carry chain is legal and the
; selector emits AddSetCarry_i32_Imm/Reg which expands to a single
; ADDW + ADCD pair.
;
; Plain MC6809 has no native i32 arith, so it stays at the byte chain
; (no `addw` appears in the asm).

target triple = "mc6809-unknown-unknown"

; i32 + immediate, sum stored, carry-out stored.
define void @uadd_with_overflow_i32_imm(i32 %a, i32* %s, i8* %c) {
  %r = call {i32, i1} @llvm.uadd.with.overflow.i32(i32 %a, i32 305419896)
  %sum = extractvalue {i32, i1} %r, 0
  %ov  = extractvalue {i32, i1} %r, 1
  store i32 %sum, i32* %s
  %ovb = zext i1 %ov to i8
  store i8 %ovb, i8* %c
  ret void
}

; HD6309-LABEL: uadd_with_overflow_i32_imm:
; HD6309:       addw
; HD6309-NEXT:  adcd
;
; MC6809-LABEL: uadd_with_overflow_i32_imm:
; MC6809-NOT:   addw

; i32 + i32 (reg), sum + carry stored.
define void @uadd_with_overflow_i32_reg(i32 %a, i32 %b, i32* %s, i8* %c) {
  %r = call {i32, i1} @llvm.uadd.with.overflow.i32(i32 %a, i32 %b)
  %sum = extractvalue {i32, i1} %r, 0
  %ov  = extractvalue {i32, i1} %r, 1
  store i32 %sum, i32* %s
  %ovb = zext i1 %ov to i8
  store i8 %ovb, i8* %c
  ret void
}

; HD6309-LABEL: uadd_with_overflow_i32_reg:
; HD6309:       addw
; HD6309:       adcd

; i32 - i32 (reg), diff + borrow stored.
define void @usub_with_overflow_i32(i32 %a, i32 %b, i32* %s, i8* %c) {
  %r = call {i32, i1} @llvm.usub.with.overflow.i32(i32 %a, i32 %b)
  %dif = extractvalue {i32, i1} %r, 0
  %ov  = extractvalue {i32, i1} %r, 1
  store i32 %dif, i32* %s
  %ovb = zext i1 %ov to i8
  store i8 %ovb, i8* %c
  ret void
}

; HD6309-LABEL: usub_with_overflow_i32:
; HD6309:       subw
; HD6309:       sbcd

; i64 add narrows to two i32 sub-chains on HD6309 — one ADDW+ADCD for
; the low half, then EXG-D,W + ADCB + ADCA + EXG + ADCD for the high
; half (carry-in via the EXG dance).  Plain MC6809 stays at 8-byte
; adds.
define void @add_i64(i64 %a, i64 %b, i64* %p) {
  %r = add i64 %a, %b
  store i64 %r, i64* %p
  ret void
}

; HD6309-LABEL: add_i64:
; HD6309:       addw
; HD6309:       adcd
; HD6309:       exg
; HD6309:       adcb
; HD6309:       adca
; HD6309:       exg
; HD6309:       adcd

declare {i32, i1} @llvm.uadd.with.overflow.i32(i32, i32)
declare {i32, i1} @llvm.usub.with.overflow.i32(i32, i32)
