; LLVM IR functions for global variable execution tests.

@counter = global i16 0
@value = global i8 0

define dso_local i16 @get_counter() local_unnamed_addr {
  %v = load i16, ptr @counter
  ret i16 %v
}

define dso_local void @set_counter(i16 noundef %v) local_unnamed_addr {
  store i16 %v, ptr @counter
  ret void
}

define dso_local void @inc_counter() local_unnamed_addr {
  %v = load i16, ptr @counter
  %v2 = add i16 %v, 1
  store i16 %v2, ptr @counter
  ret void
}

define dso_local i8 @get_value() local_unnamed_addr {
  %v = load i8, ptr @value
  ret i8 %v
}
