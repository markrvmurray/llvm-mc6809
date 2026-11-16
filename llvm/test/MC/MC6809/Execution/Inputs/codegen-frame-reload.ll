; A local read into a value, then overwritten, then the value used. The
; selector may read the slot again at the use only if nothing wrote it in
; between: here a store and a call both do. The first three functions are
; clang -O1 output, the last two clang -O0 output, for:
;
;   int n;
;   void bump(int *p) { *p = 10 + n; n += 5; }
;   int reload_after_call(int b) {
;     int a; bump(&a); int t = a; a = 7; bump(&a); return t + b;
;   }
;   ... the same with an 8-bit local and xor, and with a 16-bit and ...
;   int post_inc(int a, int b) { return b + a++; }
;   int post_dec_sub(int a, int b) { return b - a--; }

target triple = "mc6809-unknown-unknown"

@n = global i16 0, align 1

define void @bump(ptr noundef %p) noinline {
entry:
  %0 = load i16, ptr @n, align 1
  %add = add nsw i16 %0, 10
  store i16 %add, ptr %p, align 1
  %1 = load i16, ptr @n, align 1
  %add1 = add nsw i16 %1, 5
  store i16 %add1, ptr @n, align 1
  ret void
}

define void @bump8(ptr noundef %p) noinline {
entry:
  %0 = load i16, ptr @n, align 1
  %1 = trunc i16 %0 to i8
  %conv = add i8 %1, 10
  store i8 %conv, ptr %p, align 1
  %2 = load i16, ptr @n, align 1
  %add1 = add nsw i16 %2, 5
  store i16 %add1, ptr @n, align 1
  ret void
}

define i16 @reload_after_call(i16 noundef %b) {
entry:
  %a = alloca i16, align 1
  call void @bump(ptr noundef nonnull %a)
  %0 = load i16, ptr %a, align 1
  store i16 7, ptr %a, align 1
  call void @bump(ptr noundef nonnull %a)
  %add = add nsw i16 %0, %b
  ret i16 %add
}

define zeroext i8 @reload_after_call8(i8 noundef zeroext %b) {
entry:
  %a = alloca i8, align 1
  call void @bump8(ptr noundef nonnull %a)
  %0 = load i8, ptr %a, align 1
  store i8 7, ptr %a, align 1
  call void @bump8(ptr noundef nonnull %a)
  %xor3 = xor i8 %0, %b
  ret i8 %xor3
}

define i16 @and_after_call(i16 noundef %b) {
entry:
  %a = alloca i16, align 1
  call void @bump(ptr noundef nonnull %a)
  %0 = load i16, ptr %a, align 1
  store i16 7, ptr %a, align 1
  call void @bump(ptr noundef nonnull %a)
  %and = and i16 %0, %b
  ret i16 %and
}

define i16 @post_inc(i16 noundef %a, i16 noundef %b) {
entry:
  %a.addr = alloca i16, align 1
  %b.addr = alloca i16, align 1
  store i16 %a, ptr %a.addr, align 1
  store i16 %b, ptr %b.addr, align 1
  %0 = load i16, ptr %b.addr, align 1
  %1 = load i16, ptr %a.addr, align 1
  %inc = add nsw i16 %1, 1
  store i16 %inc, ptr %a.addr, align 1
  %add = add nsw i16 %0, %1
  ret i16 %add
}

define i16 @post_dec_sub(i16 noundef %a, i16 noundef %b) {
entry:
  %a.addr = alloca i16, align 1
  %b.addr = alloca i16, align 1
  store i16 %a, ptr %a.addr, align 1
  store i16 %b, ptr %b.addr, align 1
  %0 = load i16, ptr %b.addr, align 1
  %1 = load i16, ptr %a.addr, align 1
  %dec = add nsw i16 %1, -1
  store i16 %dec, ptr %a.addr, align 1
  %sub = sub nsw i16 %0, %1
  ret i16 %sub
}

