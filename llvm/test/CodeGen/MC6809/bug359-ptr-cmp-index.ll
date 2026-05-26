; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
;
; Bug #359: a pointer compare against a constant (commonly a null check) is
; legalized to G_PTRTOINT + s16 G_ICMP, which lands the value in the
; accumulator bank. The branch must NOT move the pointer into D via a
; `tfr x,d` ahead of a CMPD — it must compare in the index domain directly
; (CMPX/CMPY #imm, including #0 for a null test), since the pointer already
; lives in an index register and the hardware has CMPX/CMPY/CMPU.

; A null check: icmp eq ptr, null -> CMPX/CMPY #0, no tfr to D.
define void @ptr_eq_null(ptr %p, ptr %q) {
; CHECK-LABEL: ptr_eq_null:
; CHECK:       cmp{{[xy]}} #0
; CHECK-NOT:   tfr {{[xy]}},d
entry:
  %c = icmp eq ptr %p, null
  br i1 %c, label %z, label %nz
z:
  store i8 1, ptr %q
  br label %done
nz:
  store i8 2, ptr %q
  br label %done
done:
  ret void
}

; ne form.
define void @ptr_ne_null(ptr %p, ptr %q) {
; CHECK-LABEL: ptr_ne_null:
; CHECK:       cmp{{[xy]}} #0
; CHECK-NOT:   tfr {{[xy]}},d
entry:
  %c = icmp ne ptr %p, null
  br i1 %c, label %nz, label %z
z:
  store i8 1, ptr %q
  br label %done
nz:
  store i8 2, ptr %q
  br label %done
done:
  ret void
}

; Unsigned ordering against a non-zero constant: still index-domain (CMPX #imm).
define void @ptr_ugt_const(ptr %p, ptr %q) {
; CHECK-LABEL: ptr_ugt_const:
; CHECK:       cmp{{[xy]}} #256
; CHECK-NOT:   tfr {{[xy]}},d
entry:
  %ip = ptrtoint ptr %p to i16
  %c = icmp ugt i16 %ip, 256
  br i1 %c, label %hi, label %lo
hi:
  store i8 1, ptr %q
  br label %done
lo:
  store i8 2, ptr %q
  br label %done
done:
  ret void
}
