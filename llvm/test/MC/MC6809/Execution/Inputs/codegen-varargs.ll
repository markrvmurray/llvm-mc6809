; LLVM IR for varargs execution tests.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

declare void @llvm.va_start(ptr)
declare void @llvm.va_end(ptr)

; Sum two varargs i16 values.
define dso_local i16 @sum_va2(i16 %n, ...) {
entry:
  %ap = alloca ptr, align 1
  call void @llvm.va_start(ptr %ap)
  %v1 = va_arg ptr %ap, i16
  %v2 = va_arg ptr %ap, i16
  call void @llvm.va_end(ptr %ap)
  %r = add i16 %v1, %v2
  ret i16 %r
}
