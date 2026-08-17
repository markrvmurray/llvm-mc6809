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
