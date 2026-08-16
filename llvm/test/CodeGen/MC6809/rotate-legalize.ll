; RUN: llc -mtriple=mc6809 -O0 -verify-machineinstrs %s -o /dev/null
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs %s -o /dev/null

; Every width of rotate spelled as a funnel shift of a value with itself
; must legalize at every optimisation level.
;
; At -O0 there is no combiner to rewrite fshl(x, x, c) into a rotate, and
; nothing selects a G_FSHL, so the funnel-shift legalization has to
; recognise the rotate itself — but only below 32 bits, because from there
; up a rotate is LOWERED to a funnel shift (so its shifts become libcalls),
; and rewriting it back would cycle for ever: compiling picolibc's
; arc4random.c, thirty-three fshl.i32, hung the compiler.
;
; A rotate formed from fshl.i16 carries an i16 amount, which the rotate
; rules could not narrow.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

declare i8 @llvm.fshl.i8(i8, i8, i8)
declare i8 @llvm.fshr.i8(i8, i8, i8)
declare i16 @llvm.fshl.i16(i16, i16, i16)
declare i16 @llvm.fshr.i16(i16, i16, i16)
declare i32 @llvm.fshl.i32(i32, i32, i32)
declare i32 @llvm.fshr.i32(i32, i32, i32)

define i8 @rol8(i8 %x) {
  %r = call i8 @llvm.fshl.i8(i8 %x, i8 %x, i8 3)
  ret i8 %r
}
define i8 @ror8(i8 %x) {
  %r = call i8 @llvm.fshr.i8(i8 %x, i8 %x, i8 1)
  ret i8 %r
}
define i16 @rol16(i16 %x) {
  %r = call i16 @llvm.fshl.i16(i16 %x, i16 %x, i16 1)
  ret i16 %r
}
define i16 @ror16(i16 %x) {
  %r = call i16 @llvm.fshr.i16(i16 %x, i16 %x, i16 3)
  ret i16 %r
}
define i32 @rol32(i32 %x) {
  %r = call i32 @llvm.fshl.i32(i32 %x, i32 %x, i32 7)
  ret i32 %r
}
define i32 @ror32(i32 %x) {
  %r = call i32 @llvm.fshr.i32(i32 %x, i32 %x, i32 12)
  ret i32 %r
}
