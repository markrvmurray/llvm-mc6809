; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=HD6309
; RUN: llc -mtriple=mc6809 -mcpu=mc6809 -O2 -global-isel -global-isel-abort=1 < %s | FileCheck %s --check-prefix=MC6809
;
; Phase 3 of the i64-i32-i16-i8 plan: HD6309 G_STORE s32 legalizes
; to native STQ.  Mirror of load_i32_native_ldq.ll for the store side.

target triple = "mc6809-unknown-unknown"

define void @store_simple(i32* %p, i32 %v) {
  store i32 %v, i32* %p
  ret void
}

; HD6309-LABEL: store_simple:
; HD6309:       stq     ,
;
; MC6809-LABEL: store_simple:
; MC6809-NOT:   stq

define void @store_offset(i32* %p, i32 %v) {
  %a = getelementptr i32, i32* %p, i16 4
  store i32 %v, i32* %a
  ret void
}

; HD6309-LABEL: store_offset:
; HD6309:       stq

define void @store_pair(i32* %p, i32 %v, i32* %q, i32 %w) {
  store i32 %v, i32* %p
  store i32 %w, i32* %q
  ret void
}

; HD6309-LABEL: store_pair:
; Two i32 values are simultaneously live here and {AQ} is the whole i32
; class since the SPILL_Q retirement: one store keeps the native LDQ/STQ,
; the other folds to the two-LDD/STD slot-to-slot copy through D (the
; spiller's pressure-relief fold). What matters is one native STQ and no
; second AQ residency.
; HD6309:       std
; HD6309:       stq
