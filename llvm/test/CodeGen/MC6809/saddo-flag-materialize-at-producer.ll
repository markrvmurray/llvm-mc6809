; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -O2 %s -o - | FileCheck %s
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs %s -o /dev/null

; The overflow bit of a checked add lives in CC.V, and V is rewritten by
; the very next store (STD/STW clear it) or arithmetic. When the overflow
; result is consumed through an anyext -- as it is when the i1 is
; negated and returned, mktime's validate_structure being the case that
; failed -- the flag must be captured into a byte right after the ADC that
; produced it, not where the anyext happens to sit, because the sum is
; stored in between.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)

; CHECK-LABEL: checked_add_store:
; CHECK:         adc{{[ab]}}
; CHECK-NEXT:    tfr cc,b
; CHECK-NEXT:    lsrb
; CHECK-NEXT:    andb #1
; CHECK:         st{{[dw]}}
; CHECK:         rts
define i1 @checked_add_store(i16 %a, i16 %b, ptr %p) {
  %r = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %a, i16 %b)
  %o = extractvalue { i16, i1 } %r, 1
  %s = extractvalue { i16, i1 } %r, 0
  store i16 %s, ptr %p
  %ok = xor i1 %o, true
  ret i1 %ok
}

; The overflow bit consumed as a value -- stored as a byte, or steering a
; select -- goes through the same capture: the byte is made right after the
; ADC, before the sum's own store.

; CHECK-LABEL: checked_add_store_flag:
; CHECK:         adc{{[ab]}}
; CHECK-NEXT:    tfr cc,b
; CHECK-NEXT:    lsrb
; CHECK-NEXT:    andb #1
; CHECK:         rts
define void @checked_add_store_flag(i16 %a, i16 %b, ptr %p, ptr %q) {
  %r = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %a, i16 %b)
  %s = extractvalue { i16, i1 } %r, 0
  %o = extractvalue { i16, i1 } %r, 1
  store i16 %s, ptr %p
  %ob = zext i1 %o to i8
  store i8 %ob, ptr %q
  ret void
}

; CHECK-LABEL: checked_add_select:
; CHECK:         adc{{[ab]}}
; CHECK-NEXT:    tfr cc,b
; CHECK-NEXT:    lsrb
; CHECK-NEXT:    andb #1
; CHECK:         rts
define i16 @checked_add_select(i16 %a, i16 %b, ptr %p) {
  %r = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %a, i16 %b)
  %s = extractvalue { i16, i1 } %r, 0
  %o = extractvalue { i16, i1 } %r, 1
  store i16 %s, ptr %p
  %v = select i1 %o, i16 -1, i16 %s
  ret i16 %v
}
