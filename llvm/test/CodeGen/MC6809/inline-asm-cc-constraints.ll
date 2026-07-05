; RUN: llc -global-isel -mtriple=mc6809 -O2 %s -o - | FileCheck %s

; The 'c' and 'v' inline-asm constraints bind an i1 to the CC.C (carry) and
; CC.V (overflow) flags respectively.  On output the flag is materialised into
; a byte; on input the byte's LSB is materialised back into the flag.  This is
; the mechanism the OS-9 syscall convention (error signalled in CC.C after
; SWI2) relies on.

target triple = "mc6809-unknown-unknown"

; Carry OUT: the asm sets CC.C, we capture it into a byte (LDB #0; ADCB #0).
; CHECK-LABEL: carry_out:
; CHECK:      orcc #1
; CHECK:      ldb #0
; CHECK-NEXT: adcb #0
define i8 @carry_out() {
  %c = call i1 asm sideeffect "orcc #1", "=c"()
  %z = zext i1 %c to i8
  ret i8 %z
}

; Overflow OUT: capture CC.V into a byte (TFR CC,B; LSRB; ANDB #1).
; CHECK-LABEL: overflow_out:
; CHECK:      tfr cc,b
; CHECK-NEXT: lsrb
; CHECK-NEXT: andb #1
define i8 @overflow_out() {
  %v = call i1 asm sideeffect "", "=v"()
  %z = zext i1 %v to i8
  ret i8 %z
}

; Carry IN: place the i1 argument's LSB into CC.C (ANDB #1; LSRB) for the asm.
; CHECK-LABEL: carry_in:
; CHECK:      andb #1
; CHECK-NEXT: lsrb
define void @carry_in(i1 %f) {
  call void asm sideeffect "bcs 1f\0A1:", "c"(i1 %f)
  ret void
}
