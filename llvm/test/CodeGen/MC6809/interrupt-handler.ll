; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 < %s | FileCheck %s --check-prefixes=CHECK,M6809
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 < %s | FileCheck %s --check-prefixes=CHECK,H6309

; An interrupt handler returns with RTI. The processor has stacked its own
; registers already; what the handler must add is the direct-page registers,
; which are scratch to every other function and so may be in use by the code
; it interrupted. It saves the ones it uses, or all four when it calls
; something that might use any. clang -O2 output for:
;
;   volatile unsigned char ticks; unsigned long total;
;   volatile unsigned long step; unsigned scaled;
;   __attribute__((interrupt)) void count_only(void) { ticks++; }
;   __attribute__((interrupt)) void add_long(void) { total = total + step + 0x01020304UL; }
;   __attribute__((interrupt)) void calls_out(void) { scaled = scaled * 3; }

target triple = "mc6809-unknown-unknown"


@ticks = global i8 0, align 1
@total = global i32 0, align 1
@step = global i32 0, align 1
@scaled = global i16 0, align 1

define void @count_only() "interrupt" {
entry:
  %0 = load volatile i8, ptr @ticks, align 1
  %inc = add i8 %0, 1
  store volatile i8 %inc, ptr @ticks, align 1
  ret void
}

define void @add_long() "interrupt" {
entry:
  %0 = load i32, ptr @total, align 1
  %1 = load volatile i32, ptr @step, align 1
  %add = add i32 %0, 16909060
  %add1 = add i32 %add, %1
  store i32 %add1, ptr @total, align 1
  ret void
}

define void @calls_out() "interrupt" {
entry:
  %0 = load i16, ptr @scaled, align 1
  %mul = mul i16 %0, 3
  store i16 %mul, ptr @scaled, align 1
  ret void
}


; Nothing to save: no push, and no frame.
; CHECK-LABEL: count_only:
; CHECK-NOT:   pshs
; CHECK-NOT:   __rs
; CHECK:       rti

; A 32-bit add on the 6809 goes through the direct-page registers, so the
; ones it touches are pushed on entry and put back before the RTI. The 6309
; adds in Q and needs none.
; CHECK-LABEL: add_long:
; M6809:       ldd <__rs0
; M6809-NEXT:  pshs d
; H6309-NOT:   pshs
; H6309-NOT:   __rs
; CHECK:       rti

; A call may use any of them: all four are saved, in order, and restored in
; reverse.
; CHECK-LABEL: calls_out:
; CHECK:       ldd <__rs0
; CHECK-NEXT:  pshs d
; CHECK-NEXT:  ldd <__rs1
; CHECK-NEXT:  pshs d
; CHECK-NEXT:  ldd <__rs2
; CHECK-NEXT:  pshs d
; CHECK-NEXT:  ldd <__rs3
; CHECK-NEXT:  pshs d
; CHECK:       {{l?b?jsr|lbsr}} __mulhi3
; CHECK:       puls d
; CHECK-NEXT:  std <__rs3
; CHECK-NEXT:  puls d
; CHECK-NEXT:  std <__rs2
; CHECK-NEXT:  puls d
; CHECK-NEXT:  std <__rs1
; CHECK-NEXT:  puls d
; CHECK-NEXT:  std <__rs0
; CHECK:       rti
