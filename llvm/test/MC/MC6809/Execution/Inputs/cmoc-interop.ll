; LLVM IR functions called from CMOC driver via __gcccall convention.
; Convention: first i16 arg in X, second on stack, return in X.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

define dso_local i16 @llvm_sum(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %r = add i16 %a, %b
  ret i16 %r
}

define dso_local i16 @llvm_mul(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %r = mul i16 %a, %b
  ret i16 %r
}
