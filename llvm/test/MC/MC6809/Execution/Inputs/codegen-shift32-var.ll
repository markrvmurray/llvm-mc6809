; LLVM IR for i32 variable shift execution tests (via codegen → libcall).

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

define dso_local i32 @shl32(i32 noundef %a, i8 noundef %n) {
  %n32 = zext i8 %n to i32
  %r = shl i32 %a, %n32
  ret i32 %r
}
