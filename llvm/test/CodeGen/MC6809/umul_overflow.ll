; RUN: llc -global-isel -global-isel-abort=1 -O0 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O1 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O3 -mtriple=mc6809 < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 < %s | FileCheck %s
;
; Unsigned multiply with overflow (__builtin_mul_overflow; calloc, fread and
; fwrite check a size this way) is never a division -- testing
; a > UINT_MAX / b called __udivhi3 or __udivsi3, which cost more than the
; multiply being checked. One MUL at 8 bits; __umulohi4 (three MULs) at 16;
; at 32 bits the multiply itself plus __umulohi4 on the cross product. The
; answers are checked on both processors by
; test/MC/MC6809/Execution/codegen-umulo.s.

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: umul8_sat:
; CHECK-NOT: lbsr
; CHECK: mul
; CHECK-NOT: lbsr
; CHECK: {{rts|puls .*,pc}}
define i8 @umul8_sat(i8 %a, i8 %b) {
  %r = call { i8, i1 } @llvm.umul.with.overflow.i8(i8 %a, i8 %b)
  %v = extractvalue { i8, i1 } %r, 0
  %o = extractvalue { i8, i1 } %r, 1
  %s = select i1 %o, i8 -1, i8 %v
  ret i8 %s
}

; CHECK-LABEL: umul16_sat:
; CHECK-NOT: lbsr
; CHECK: lbsr __umulohi4
; CHECK-NOT: lbsr
; CHECK: {{rts|puls .*,pc}}
define i16 @umul16_sat(i16 %a, i16 %b) {
  %r = call { i16, i1 } @llvm.umul.with.overflow.i16(i16 %a, i16 %b)
  %v = extractvalue { i16, i1 } %r, 0
  %o = extractvalue { i16, i1 } %r, 1
  %s = select i1 %o, i16 -1, i16 %v
  ret i16 %s
}

; CHECK-LABEL: umul32_sat:
; CHECK-NOT: lbsr
; CHECK-DAG: lbsr __mulsi3
; CHECK-DAG: lbsr __umulohi4
; CHECK-NOT: lbsr
; CHECK: {{rts|puls .*,pc}}
define i32 @umul32_sat(i32 %a, i32 %b) {
  %r = call { i32, i1 } @llvm.umul.with.overflow.i32(i32 %a, i32 %b)
  %v = extractvalue { i32, i1 } %r, 0
  %o = extractvalue { i32, i1 } %r, 1
  %s = select i1 %o, i32 -1, i32 %v
  ret i32 %s
}

; s64 is a call to __umulodi4 (see stack-realign.ll).

declare { i8,  i1 } @llvm.umul.with.overflow.i8(i8,  i8)
declare { i16, i1 } @llvm.umul.with.overflow.i16(i16, i16)
declare { i32, i1 } @llvm.umul.with.overflow.i32(i32, i32)
