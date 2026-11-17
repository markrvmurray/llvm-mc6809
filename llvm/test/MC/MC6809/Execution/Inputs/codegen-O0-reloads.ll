; Two shapes that -O0 once compiled wrongly, each by deleting a reload while
; the register it reloaded was still needed. clang -O0 output for:
;
;   union ptr_or_int { char *p; int i; };
;   int value_after_compare(char *p) {
;     union ptr_or_int u; int v;
;     u.p = p; if ((v = u.i) == 100) return 1; callee(); return v;
;   }
;   char add_through_local_pointer(char *o) {
;     char a = 0; char *p = &a; char t = o[1]; *p += t; return a;
;   }

target triple = "mc6809-unknown-unknown"

%union.ptr_or_int = type { ptr }

define void @callee() {
entry:
  ret void
}

define i16 @value_after_compare(ptr noundef %p) {
entry:
  %retval = alloca i16, align 1
  %p.addr = alloca ptr, align 1
  %u = alloca %union.ptr_or_int, align 1
  %v = alloca i16, align 1
  store ptr %p, ptr %p.addr, align 1
  %0 = load ptr, ptr %p.addr, align 1
  store ptr %0, ptr %u, align 1
  %1 = load i16, ptr %u, align 1
  store i16 %1, ptr %v, align 1
  %cmp = icmp eq i16 %1, 100
  br i1 %cmp, label %if.then, label %if.end

if.then:                                          ; preds = %entry
  store i16 1, ptr %retval, align 1
  br label %return

if.end:                                           ; preds = %entry
  call void @callee()
  %2 = load i16, ptr %v, align 1
  store i16 %2, ptr %retval, align 1
  br label %return

return:                                           ; preds = %if.end, %if.then
  %3 = load i16, ptr %retval, align 1
  ret i16 %3
}

define zeroext i8 @add_through_local_pointer(ptr noundef %o) {
entry:
  %o.addr = alloca ptr, align 1
  %a = alloca i8, align 1
  %p = alloca ptr, align 1
  %t = alloca i8, align 1
  store ptr %o, ptr %o.addr, align 1
  store i8 0, ptr %a, align 1
  store ptr %a, ptr %p, align 1
  %0 = load ptr, ptr %o.addr, align 1
  %arrayidx = getelementptr inbounds i8, ptr %0, i16 1
  %1 = load i8, ptr %arrayidx, align 1
  store i8 %1, ptr %t, align 1
  %2 = load i8, ptr %t, align 1
  %conv = zext i8 %2 to i16
  %3 = load ptr, ptr %p, align 1
  %4 = load i8, ptr %3, align 1
  %conv1 = zext i8 %4 to i16
  %add = add nsw i16 %conv1, %conv
  %conv2 = trunc i16 %add to i8
  store i8 %conv2, ptr %3, align 1
  %5 = load i8, ptr %a, align 1
  ret i8 %5
}

