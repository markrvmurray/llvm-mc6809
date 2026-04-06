; LLVM IR for byval parameter execution tests.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

%struct.Point = type { i16, i16 }

define dso_local i16 @get_x(ptr byval(%struct.Point) %p) {
  %x = load i16, ptr %p, align 1
  ret i16 %x
}

define dso_local i16 @get_y(ptr byval(%struct.Point) %p) {
  %yptr = getelementptr inbounds %struct.Point, ptr %p, i16 0, i32 1
  %y = load i16, ptr %yptr, align 1
  ret i16 %y
}

define dso_local i16 @sum_point(ptr byval(%struct.Point) %p) {
  %x = load i16, ptr %p, align 1
  %yptr = getelementptr inbounds %struct.Point, ptr %p, i16 0, i32 1
  %y = load i16, ptr %yptr, align 1
  %r = add i16 %x, %y
  ret i16 %r
}
