; RUN: llc -mtriple=mc6809-unknown-os9 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: an OS-9 program module is placed at run time and its body is
; read-only and shared, so a constant that holds a pointer cannot live there
; -- its value is not known until the module and the data area are placed.
; Such constants go to a writable-image section (.data.rel.ro*), which the
; module's linker script copies into the data area and the CRT rebases, and
; the code reads them U-relatively.  Pointer-free constants stay in the body
; and are read PC-relatively.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-os9"

@.str = private unnamed_addr constant [2 x i8] c"a\00", align 1
@.str.1 = private unnamed_addr constant [3 x i8] c"bb\00", align 1

; A table of pointers: data area.
@ptrtab = constant [2 x ptr] [ptr @.str, ptr @.str.1], align 1
; A pointer-typed constant that happens to be null: still the data area, so
; that its address and the table's agree with the section it was emitted to.
@nullptr = constant ptr null, align 1
; No pointer anywhere in the type: the body.
@ints = constant [3 x i16] [i16 1, i16 2, i16 3], align 1

; CHECK-LABEL: gettab:
; CHECK: leax mc6809_os9_data(ptrtab),u
define ptr @gettab(i16 %i) {
  %p = getelementptr [2 x ptr], ptr @ptrtab, i16 0, i16 %i
  %v = load ptr, ptr %p
  ret ptr %v
}

define ptr @getnull() {
  %v = load ptr, ptr @nullptr
  ret ptr %v
}

; CHECK-LABEL: getint:
; CHECK: leax ints,pc
; CHECK-NOT: mc6809_os9_data(ints)
define i16 @getint(i16 %i) {
  %p = getelementptr [3 x i16], ptr @ints, i16 0, i16 %i
  %v = load i16, ptr %p
  ret i16 %v
}

; The pointer table is emitted to a writable-image section, the integers to
; the read-only body.
; CHECK: .section .data.rel.ro
; CHECK: ptrtab:
; CHECK: nullptr:
; CHECK: .section .rodata
; CHECK: ints:
