; A value adjusted by +1 or -1 chosen in two predecessors, then stored back.
; MC6809IncDecPhi moves the load, the add and the store up into the
; predecessors; it may do that only past instructions that neither write nor
; read memory. clang -O1 output for:
;
;   char cell;
;   void bump_after_call(char *p, char c) {
;     char d; if (c) { on_true(); d = 1; } else { on_false(); d = -1; }
;     writer();              /* cell = 100 */
;     *p += d;
;   }
;   char read_then_bump(char *p, char c) {
;     char d; ... as above ...
;     char before = cell; *p += d; return before;
;   }

target triple = "mc6809-unknown-unknown"

@cell = global i8 0, align 1

define void @on_true() noinline {
entry:
  ret void
}

define void @on_false() noinline {
entry:
  ret void
}

define void @writer() noinline {
entry:
  store i8 100, ptr @cell, align 1
  ret void
}

define void @bump_after_call(ptr noundef %p, i8 noundef zeroext %c) {
entry:
  %tobool.not = icmp eq i8 %c, 0
  %. = select i1 %tobool.not, i8 -1, i8 1
  tail call void @writer()
  %0 = load i8, ptr %p, align 1
  %add = add i8 %0, %.
  store i8 %add, ptr %p, align 1
  ret void
}

define zeroext i8 @read_then_bump(ptr noundef %p, i8 noundef zeroext %c) {
entry:
  %tobool.not = icmp eq i8 %c, 0
  %. = select i1 %tobool.not, i8 -1, i8 1
  %0 = load i8, ptr @cell, align 1
  %1 = load i8, ptr %p, align 1
  %add = add i8 %1, %.
  store i8 %add, ptr %p, align 1
  ret i8 %0
}

