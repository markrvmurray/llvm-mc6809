; Leading-zero counts at 16 bits and narrower, zero defined and zero poison.
target triple = "mc6809"

declare i16 @llvm.ctlz.i16(i16, i1)
declare i8 @llvm.ctlz.i8(i8, i1)

define i16 @clz16(i16 %a) {
  %r = call i16 @llvm.ctlz.i16(i16 %a, i1 false)
  ret i16 %r
}

define i16 @clz16_nz(i16 %a) {
  %r = call i16 @llvm.ctlz.i16(i16 %a, i1 true)
  ret i16 %r
}

define i8 @clz8(i8 %a) {
  %r = call i8 @llvm.ctlz.i8(i8 %a, i1 false)
  ret i8 %r
}
