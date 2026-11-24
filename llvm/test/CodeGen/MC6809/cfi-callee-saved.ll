; The CFI says where the frame starts and where each saved register went.
;
; A call leaves the CFA at S + 2 with the return address at CFA - 2.  The
; prologue says, as each push is made, where each callee-saved register went
; -- as an offset from S just after the push (.cfi_rel_offset), which the
; assembler turns into an offset from the CFA -- and how far S has moved.  A
; push stores its highest register first: `pshs u,y` leaves U at CFA-4 and Y
; at CFA-6.
;
; An interrupt handler is entered with the machine state stacked instead: an
; IRQ, NMI or software interrupt stacks CC, A, B, DP, X, Y, U and PC (and E
; and F after B on a 6309 in native mode, the mode a 6309 handler assumes),
; a FIRQ only CC and PC.  The CFA is S before the interrupt, every register of
; the interrupted code is described at the slot the processor put it in (D
; at A's), and the handler's own copies of those are not a second location.
; The direct-page registers it saves are memory under DWARF numbers no
; debugger knows: their pushes move the CFA but are not described as saves.
;
; RUN: llc -mtriple=mc6809 -O2 < %s | FileCheck %s --check-prefixes=CHECK,M6809
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 < %s | FileCheck %s --check-prefixes=CHECK,H6309
; RUN: llc -mtriple=mc6809 -O2 -filetype=obj < %s | llvm-dwarfdump --debug-frame - | FileCheck %s --check-prefix=ROWS

declare i16 @h(i16)
declare void @g(i16)
@n = global i16 0

; CHECK-LABEL: save_uy:
; CHECK:       .cfi_def_cfa s, 2
; CHECK-NEXT:  .cfi_offset pc, -2
; CHECK-NEXT:  pshs u,y
; CHECK-NEXT:  .cfi_adjust_cfa_offset 4
; CHECK-NEXT:  .cfi_rel_offset u, 2
; CHECK-NEXT:  .cfi_rel_offset y, 0
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
; CHECK-NEXT:  .cfi_adjust_cfa_offset 2
; CHECK-NEXT:  .cfi_rel_offset u, 0
define i16 @save_u(i16 %a) uwtable {
  %x = call i16 @h(i16 %a)
  %y = add i16 %x, %a
  ret i16 %y
}

; CHECK-LABEL: irq:
; M6809:       .cfi_def_cfa s, 12
; M6809-NEXT:  .cfi_offset cc, -12
; M6809-NEXT:  .cfi_offset a, -11
; M6809-NEXT:  .cfi_offset d, -11
; M6809-NEXT:  .cfi_offset b, -10
; H6309:       .cfi_def_cfa s, 14
; H6309-NEXT:  .cfi_offset cc, -14
; H6309-NEXT:  .cfi_offset a, -13
; H6309-NEXT:  .cfi_offset d, -13
; H6309-NEXT:  .cfi_offset b, -12
; H6309-NEXT:  .cfi_offset e, -11
; H6309-NEXT:  .cfi_offset f, -10
; CHECK-NEXT:  .cfi_offset dp, -9
; CHECK-NEXT:  .cfi_offset x, -8
; CHECK-NEXT:  .cfi_offset y, -6
; CHECK-NEXT:  .cfi_offset u, -4
; CHECK-NEXT:  .cfi_offset pc, -2
; CHECK:       pshs d
; CHECK-NEXT:  .cfi_adjust_cfa_offset 2
; CHECK-NOT:   .cfi_{{rel_offset|offset}}
; CHECK:       rti
define void @irq() uwtable "interrupt" {
  %v = load volatile i16, ptr @n
  %w = add i16 %v, 1
  store volatile i16 %w, ptr @n
  call void @g(i16 %w)
  ret void
}

; CHECK-LABEL: firq:
; CHECK:       .cfi_def_cfa s, 3
; CHECK-NEXT:  .cfi_offset cc, -3
; CHECK-NEXT:  .cfi_offset pc, -2
; CHECK:       .cfi_rel_offset
; CHECK:       rti
define void @firq() uwtable "interrupt"="firq" {
  %v = load volatile i16, ptr @n
  %w = add i16 %v, 1
  store volatile i16 %w, ptr @n
  ret void
}

; ROWS: CFA=SS+6: IY=[CFA-6], SU=[CFA-4], PC=[CFA-2]
; ROWS: CFA=SS+12: AD=[CFA-11], IX=[CFA-8], IY=[CFA-6], SU=[CFA-4], PC=[CFA-2], AA=[CFA-11], AB=[CFA-10], CC=[CFA-12], DP=[CFA-9]
