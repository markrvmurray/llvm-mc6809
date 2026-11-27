; RUN: opt -passes=instcombine -S %s | FileCheck %s --check-prefix=OS9
; RUN: sed -e 's/mc6809-unknown-os9/mc6809-unknown-unknown/' %s \
; RUN:   | opt -passes=instcombine -S | FileCheck %s --check-prefix=LF

; OS-9's C library ends a line with a carriage return: puts appends one, so
; the printf-to-puts rewrites take a trailing CR, not LF, for the newline
; puts would add, and puts("") becomes putchar of CR.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-os9"

@cr_line = constant [4 x i8] c"hi\0D\00"
@lf_line = constant [4 x i8] c"hi\0A\00"
@empty = constant [1 x i8] zeroinitializer
@fmt_s = constant [3 x i8] c"%s\00"
@fmt_s_cr = constant [4 x i8] c"%s\0D\00"
@fmt_s_lf = constant [4 x i8] c"%s\0A\00"

declare i16 @printf(ptr, ...)
declare i16 @puts(ptr)

; OS9-LABEL: define void @printf_cr_line(
; OS9-NEXT:    {{.*}}call i16 @puts(ptr {{.*}}@str)
; LF-LABEL:  define void @printf_cr_line(
; LF-NEXT:     {{.*}}call i16 (ptr, ...) @printf(ptr {{.*}}@cr_line)
define void @printf_cr_line() {
  call i16 (ptr, ...) @printf(ptr @cr_line)
  ret void
}

; OS9-LABEL: define void @printf_lf_line(
; OS9-NEXT:    {{.*}}call i16 (ptr, ...) @printf(ptr {{.*}}@lf_line)
; LF-LABEL:  define void @printf_lf_line(
; LF-NEXT:     {{.*}}call i16 @puts(ptr {{.*}}@str)
define void @printf_lf_line() {
  call i16 (ptr, ...) @printf(ptr @lf_line)
  ret void
}

; OS9-LABEL: define void @printf_s_cr_line(
; OS9-NEXT:    {{.*}}call i16 @puts(ptr {{.*}}@str.1)
; LF-LABEL:  define void @printf_s_cr_line(
; LF-NEXT:     {{.*}}call i16 (ptr, ...) @printf(ptr {{.*}}@fmt_s, ptr nonnull @cr_line)
define void @printf_s_cr_line() {
  call i16 (ptr, ...) @printf(ptr @fmt_s, ptr @cr_line)
  ret void
}

; OS9-LABEL: define void @printf_fmt_s_cr(
; OS9-NEXT:    {{.*}}call i16 @puts(ptr {{.*}}%s)
; LF-LABEL:  define void @printf_fmt_s_cr(
; LF-NEXT:     {{.*}}call i16 (ptr, ...) @printf(ptr {{.*}}@fmt_s_cr, ptr %s)
define void @printf_fmt_s_cr(ptr %s) {
  call i16 (ptr, ...) @printf(ptr @fmt_s_cr, ptr %s)
  ret void
}

; OS9-LABEL: define void @printf_fmt_s_lf(
; OS9-NEXT:    {{.*}}call i16 (ptr, ...) @printf(ptr {{.*}}@fmt_s_lf, ptr %s)
; LF-LABEL:  define void @printf_fmt_s_lf(
; LF-NEXT:     {{.*}}call i16 @puts(ptr {{.*}}%s)
define void @printf_fmt_s_lf(ptr %s) {
  call i16 (ptr, ...) @printf(ptr @fmt_s_lf, ptr %s)
  ret void
}

; OS9-LABEL: define void @puts_empty(
; OS9-NEXT:    {{.*}}call i16 @putchar(i16 13)
; LF-LABEL:  define void @puts_empty(
; LF-NEXT:     {{.*}}call i16 @putchar(i16 10)
define void @puts_empty() {
  call i16 @puts(ptr @empty)
  ret void
}
