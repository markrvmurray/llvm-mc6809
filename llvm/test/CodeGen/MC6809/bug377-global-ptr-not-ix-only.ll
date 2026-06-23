; RUN: llc -O2 -mtriple=mc6809 < %s | FileCheck %s

; Pointer values loaded from globals go through the G_GLOBAL_VALUE extended
; pointer-load fold (LDXe). That value must live in the wide INDEX16 class
; (IX/IY/spill), NOT the 1-register IX-only class — otherwise keeping several
; such pointers live at once (as malloc's free-list walk does, once LTO inlines
; memset/sbrk into it) exhausts the 1-wide class and greedy regalloc fails with
; "ran out of registers". This compiles five global-loaded pointers and keeps
; them all live; pre-fix it failed register allocation.

@g0 = external global ptr
@g1 = external global ptr
@g2 = external global ptr
@g3 = external global ptr
@g4 = external global ptr

; CHECK-LABEL: many_global_ptrs:
define i16 @many_global_ptrs(i16 %i) {
  %p0 = load ptr, ptr @g0
  %p1 = load ptr, ptr @g1
  %p2 = load ptr, ptr @g2
  %p3 = load ptr, ptr @g3
  %p4 = load ptr, ptr @g4
  %a0 = getelementptr i16, ptr %p0, i16 %i
  %a1 = getelementptr i16, ptr %p1, i16 %i
  %a2 = getelementptr i16, ptr %p2, i16 %i
  %a3 = getelementptr i16, ptr %p3, i16 %i
  %a4 = getelementptr i16, ptr %p4, i16 %i
  %v0 = load i16, ptr %a0
  %v1 = load i16, ptr %a1
  %v2 = load i16, ptr %a2
  %v3 = load i16, ptr %a3
  %v4 = load i16, ptr %a4
  %s0 = add i16 %v0, %v1
  %s1 = add i16 %s0, %v2
  %s2 = add i16 %s1, %v3
  %s3 = add i16 %s2, %v4
  ret i16 %s3
}
