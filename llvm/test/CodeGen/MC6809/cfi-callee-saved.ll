; The prologue says where it saved each callee-saved register, so that a
; debugger unwinding past this function finds the caller's Y and U rather
; than carrying this function's values up the stack.  The CFA is S before the
; call pushed the return address (S + 2 at entry), and a push stores its
; highest register first: `pshs u,y` leaves U at CFA-4 and Y at CFA-6.
; An interrupt handler saves its direct-page registers through D, which
; stores B (the second of a pair) above A.
;
; RUN: llc -mtriple=mc6809 -O2 < %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -O2 -filetype=obj < %s | llvm-dwarfdump --debug-frame - | FileCheck %s --check-prefix=ROWS

declare i16 @h(i16)
declare void @g(i16)
@n = global i16 0

; CHECK-LABEL: save_uy:
; CHECK:       .cfi_def_cfa s, 2
; CHECK-NEXT:  .cfi_offset pc, -2
; CHECK-NEXT:  pshs u,y
; CHECK-NEXT:  .cfi_offset u, -4
; CHECK-NEXT:  .cfi_offset y, -6
; CHECK-NEXT:  .cfi_def_cfa_offset 6
define i16 @save_uy(i16 %a, i16 %b, i16 %c) uwtable {
  %x = call i16 @h(i16 %a)
  %y = call i16 @h(i16 %b)
  call void @g(i16 %c)
  call void @g(i16 %x)
  %s = add i16 %x, %y
  %t = add i16 %s, %a
  ret i16 %t
}

; CHECK-LABEL: save_u:
; CHECK:       pshs u
; CHECK-NEXT:  .cfi_offset u, -4
; CHECK-NEXT:  .cfi_def_cfa_offset 4
define i16 @save_u(i16 %a) uwtable {
  %x = call i16 @h(i16 %a)
  %y = add i16 %x, %a
  ret i16 %y
}

; CHECK-LABEL: irq:
; CHECK:       pshs d
; CHECK-NEXT:  .cfi_offset rs0, -4
; CHECK:       lda <__rc0
; CHECK-NEXT:  ldb <__rc1
; CHECK-NEXT:  pshs d
; CHECK-NEXT:  .cfi_offset rc1, -11
; CHECK-NEXT:  .cfi_offset rc0, -12
define void @irq() uwtable "interrupt" {
  %v = load volatile i16, ptr @n
  %w = add i16 %v, 1
  store volatile i16 %w, ptr @n
  call void @g(i16 %w)
  ret void
}

; ROWS: U=[CFA-4]
; ROWS: Y=[CFA-6]
