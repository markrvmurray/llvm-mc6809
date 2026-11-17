; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s

; A signed add whose overflow flag is wanted as a value carries an implicit
; def of the byte to copy V into, and its expansion makes the copy right
; after the ADC. In a leaf with a static frame its frame-slot operand is
; rewritten to the static-stack form, and that rewrite dropped the tag: the
; copy was never made, and the branch tested whatever was in the byte.
; From picolibc's test-saddo-overflow.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: saddo16:
; CHECK:       adc{{[ab]}} {{.*}}saddo16_sstk
; CHECK-NEXT:  tfr cc,{{[ab]}}
; CHECK-NEXT:  lsr{{[ab]}}
; CHECK-NEXT:  and{{[ab]}} #1
define internal i16 @saddo16(i16 %x, i16 %y) noinline norecurse {
entry:
  %ny = sub i16 0, %y
  %r = call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %x, i16 %ny)
  %ov = extractvalue { i16, i1 } %r, 1
  %s = extractvalue { i16, i1 } %r, 0
  %res = select i1 %ov, i16 -999, i16 %s
  ret i16 %res
}

define i16 @main() {
  %a = call i16 @saddo16(i16 -32768, i16 1)
  ret i16 %a
}

declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)
