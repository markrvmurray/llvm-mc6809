; LLVM IR for initialized .data globals execution test.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

@counter = dso_local global i16 42, align 1
@flag = dso_local global i8 1, align 1

define dso_local i16 @get_counter() {
  %v = load i16, ptr @counter, align 1
  ret i16 %v
}

define dso_local i8 @get_flag() {
  %v = load i8, ptr @flag, align 1
  ret i8 %v
}

define dso_local void @inc_counter() {
  %v = load i16, ptr @counter, align 1
  %v1 = add i16 %v, 1
  store i16 %v1, ptr @counter, align 1
  ret void
}
