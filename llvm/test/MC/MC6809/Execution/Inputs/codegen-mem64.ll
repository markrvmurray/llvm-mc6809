; LLVM IR functions for i64 load/store execution tests.
;
; Exercises the legalizer's i64-load/store splitting + the
; offset-relaxation pass installed by bug #149 Class 1, by reading
; and writing i64 through:
;   - a stack slot (alloca + store + load)
;   - a global i64 in the .data region
;   - an i64 struct field at non-zero offset
;   - an i64 array element with VARIABLE index
;
; The harness passes the input via pointer; each wrapper performs
; one of the four memory shapes and stores the resulting i64 to
; the result pointer.

target triple = "mc6809-unknown-unknown"

@g64 = dso_local global i64 0

%mixed = type { i8, i64, i16 }

; Stack: store input to alloca, load it back (round-trip).
define dso_local void @stack64_w(ptr %r, ptr %ip) {
  %a = alloca i64, align 1
  %v = load i64, ptr %ip, align 1
  store i64 %v, ptr %a, align 1
  %x = load i64, ptr %a, align 1
  store i64 %x, ptr %r, align 1
  ret void
}

; Global: store input to @g64, load it back.
define dso_local void @global64_w(ptr %r, ptr %ip) {
  %v = load i64, ptr %ip, align 1
  store i64 %v, ptr @g64, align 1
  %x = load i64, ptr @g64, align 1
  store i64 %x, ptr %r, align 1
  ret void
}

; Struct field: store input to s.f64 (offset 1, after the i8), reload.
define dso_local void @struct64_w(ptr %r, ptr %ip) {
  %s = alloca %mixed, align 1
  %v = load i64, ptr %ip, align 1
  %p = getelementptr inbounds %mixed, ptr %s, i32 0, i32 1
  store i64 %v, ptr %p, align 1
  %x = load i64, ptr %p, align 1
  store i64 %x, ptr %r, align 1
  ret void
}

; Array with VARIABLE index: arr[idx] = input; result = arr[idx]
define dso_local void @array64_w(ptr %r, ptr %ip, i16 %idx) {
  %arr = alloca [4 x i64], align 1
  %v = load i64, ptr %ip, align 1
  %p = getelementptr inbounds [4 x i64], ptr %arr, i32 0, i16 %idx
  store i64 %v, ptr %p, align 1
  %x = load i64, ptr %p, align 1
  store i64 %x, ptr %r, align 1
  ret void
}
