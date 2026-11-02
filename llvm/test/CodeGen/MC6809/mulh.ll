; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefix=O2
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefix=HD6309
; RUN: llc -mtriple=mc6809 -O0 < %s | FileCheck %s --check-prefix=O0

; InstCombine forms llvm.[su]mulh from (ext(a) * ext(b)) >> width. No runtime
; library function implements them, so the widths above i8 widen back to the
; multiply. Execution/codegen-mulh.s checks the values.

target triple = "mc6809-unknown-unknown"

declare i8 @llvm.smulh.i8(i8, i8)
declare i16 @llvm.umulh.i16(i16, i16)
declare i16 @llvm.smulh.i16(i16, i16)
declare i32 @llvm.umulh.i32(i32, i32)
declare i32 @llvm.smulh.i32(i32, i32)

; MUL is unsigned, so a signed high multiply must not use it directly.
; O2-LABEL: smulh8:
; O2: __mulhi3
; HD6309-LABEL: smulh8:
; HD6309: __mulhi3
; O0-LABEL: smulh8:
; O0: __mulhi3
define i8 @smulh8(i8 %a, i8 %b) {
  %r = call i8 @llvm.smulh.i8(i8 %a, i8 %b)
  ret i8 %r
}

; O2-LABEL: umulh16:
; O2: __mulsi3
; HD6309-LABEL: umulh16:
; HD6309: __mulsi3
; O0-LABEL: umulh16:
; O0: __mulsi3
define i16 @umulh16(i16 %a, i16 %b) {
  %r = call i16 @llvm.umulh.i16(i16 %a, i16 %b)
  ret i16 %r
}

; O2-LABEL: smulh16:
; O2: __mulsi3
; HD6309-LABEL: smulh16:
; HD6309: __mulsi3
; O0-LABEL: smulh16:
; O0: __mulsi3
define i16 @smulh16(i16 %a, i16 %b) {
  %r = call i16 @llvm.smulh.i16(i16 %a, i16 %b)
  ret i16 %r
}

; O2-LABEL: umulh32:
; O2: __muldi3
; HD6309-LABEL: umulh32:
; HD6309: __muldi3
; O0-LABEL: umulh32:
; O0: __muldi3
define i32 @umulh32(i32 %a, i32 %b) {
  %r = call i32 @llvm.umulh.i32(i32 %a, i32 %b)
  ret i32 %r
}

; O2-LABEL: smulh32:
; O2: __muldi3
; HD6309-LABEL: smulh32:
; HD6309: __muldi3
; O0-LABEL: smulh32:
; O0: __muldi3
define i32 @smulh32(i32 %a, i32 %b) {
  %r = call i32 @llvm.smulh.i32(i32 %a, i32 %b)
  ret i32 %r
}
