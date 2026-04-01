; LLVM IR for pointer/array execution tests.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

; Array indexing: return a[i]
define dso_local i16 @array_get(ptr nocapture noundef readonly %a, i16 noundef %i) {
  %ptr = getelementptr i16, ptr %a, i16 %i
  %val = load i16, ptr %ptr, align 1
  ret i16 %val
}

; Array store: a[i] = val
define dso_local void @array_set(ptr nocapture noundef %a, i16 noundef %i, i16 noundef %val) {
  %ptr = getelementptr i16, ptr %a, i16 %i
  store i16 %val, ptr %ptr, align 1
  ret void
}

; Pointer dereference: *p
define dso_local i16 @deref(ptr nocapture noundef readonly %p) {
  %val = load i16, ptr %p, align 1
  ret i16 %val
}

; Pointer write: *p = val
define dso_local void @deref_write(ptr nocapture noundef %p, i16 noundef %val) {
  store i16 %val, ptr %p, align 1
  ret void
}

; Byte array indexing: return a[i] (i8)
define dso_local i8 @byte_get(ptr nocapture noundef readonly %a, i16 noundef %i) {
  %ptr = getelementptr i8, ptr %a, i16 %i
  %val = load i8, ptr %ptr, align 1
  ret i8 %val
}

; Struct-like access: return p->field2 (offset 4 from pointer)
define dso_local i16 @struct_field(ptr nocapture noundef readonly %p) {
  %field = getelementptr i8, ptr %p, i16 4
  %val = load i16, ptr %field, align 1
  ret i16 %val
}
