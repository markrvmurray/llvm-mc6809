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

; Sum N varargs i16 values (up to 4).
define dso_local i16 @sum_va4(i16 %n, ...) {
entry:
  %ap = alloca ptr, align 1
  call void @llvm.va_start(ptr %ap)
  %v1 = va_arg ptr %ap, i16
  %v2 = va_arg ptr %ap, i16
  %v3 = va_arg ptr %ap, i16
  %v4 = va_arg ptr %ap, i16
  call void @llvm.va_end(ptr %ap)
  %s1 = add i16 %v1, %v2
  %s2 = add i16 %s1, %v3
  %s3 = add i16 %s2, %v4
  ret i16 %s3
}

; Return just the Nth vararg (1-based).
define dso_local i16 @get_nth_va(i16 %n, ...) {
entry:
  %ap = alloca ptr, align 1
  call void @llvm.va_start(ptr %ap)
  %cmp1 = icmp sgt i16 %n, 1
  br i1 %cmp1, label %skip1, label %done

skip1:
  %skip1v = va_arg ptr %ap, i16
  %cmp2 = icmp sgt i16 %n, 2
  br i1 %cmp2, label %skip2, label %done

skip2:
  %skip2v = va_arg ptr %ap, i16
  %cmp3 = icmp sgt i16 %n, 3
  br i1 %cmp3, label %skip3, label %done

skip3:
  %skip3v = va_arg ptr %ap, i16
  br label %done

done:
  %result = va_arg ptr %ap, i16
  call void @llvm.va_end(ptr %ap)
  ret i16 %result
}
