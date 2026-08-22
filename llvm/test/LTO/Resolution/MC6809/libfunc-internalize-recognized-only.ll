;; Bitcode definitions of library functions that the optimizer only ever
;; recognizes are internalized like any other symbol; only those it can
;; introduce a call to on its own must stay external for the whole pipeline.
;; Here printf's caller ignores its result, so once printf is internal its
;; return value is dead and vfprintf, called only from printf, loses its
;; result and the accounting that produced it. puts can be introduced by the
;; optimizer (printf of a plain string) so its definition, unused or not, must
;; keep its external signature.

; RUN: opt %s -o %t.o -mtriple mc6809-unknown-unknown
; RUN: llvm-lto2 run -o %t.lto.o -save-temps %t.o \
; RUN:   -r %t.o,_start,plx \
; RUN:   -r %t.o,printf,pl \
; RUN:   -r %t.o,puts,pl \
; RUN:   -r %t.o,__stdout,pl \
; RUN:   -r %t.o,__flush,pl \
; RUN:   -r %t.o,__putc,x
; RUN: llvm-dis %t.lto.o.0.2.internalize.bc -o - | FileCheck %s --check-prefix=INTERNALIZE
; RUN: llvm-dis %t.lto.o.0.4.opt.bc -o - | FileCheck %s --check-prefix=OPT

;; printf is recognized-only: internalized. puts may be introduced: kept
;; external and marked so the target can drop it once no more calls can appear.
; INTERNALIZE: define internal i16 @printf(
; INTERNALIZE: define internal fastcc i16 @vfprintf(
; INTERNALIZE: define dso_local i16 @puts({{.*}} partition "contingent"

;; With printf internal its unused result is removed, and vfprintf's with it;
;; printf itself is inlined into its only caller. puts survives untouched.
; OPT: define internal fastcc void @vfprintf(
; OPT-NOT: define {{.*}}@printf(
; OPT: define dso_local {{.*}}i16 @puts({{.*}} partition "contingent"

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

@__stdout = global i8 0

declare i16 @__putc(i8, ptr)

define internal fastcc i16 @vfprintf(ptr %fmt, ptr %ap) noinline {
entry:
  br label %loop
loop:
  %n = phi i16 [ 0, %entry ], [ %n.next, %loop ]
  %p = phi ptr [ %fmt, %entry ], [ %p.next, %loop ]
  %c = load i8, ptr %p
  %r = call i16 @__putc(i8 %c, ptr @__stdout)
  %n.next = add i16 %n, 1
  %p.next = getelementptr i8, ptr %p, i16 1
  %done = icmp eq i8 %c, 0
  br i1 %done, label %exit, label %loop
exit:
  ret i16 %n
}

define i16 @printf(ptr %fmt, ...) noinline {
  %ap = alloca ptr
  call void @llvm.va_start(ptr %ap)
  %r = call fastcc i16 @vfprintf(ptr %fmt, ptr %ap)
  call void @llvm.va_end(ptr %ap)
  ret i16 %r
}

define i16 @puts(ptr %s) noinline {
  call void asm sideeffect "", ""()
  ret i16 0
}

define void @__flush() noinline {
  ret void
}

@fmt = private constant [4 x i8] c"%d\0A\00"

define i16 @_start(i16 %v) {
  call i16 (ptr, ...) @printf(ptr @fmt, i16 %v)
  ret i16 0
}

declare void @llvm.va_start(ptr)
declare void @llvm.va_end(ptr)
