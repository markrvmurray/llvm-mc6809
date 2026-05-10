; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Bug #267 sentinel: when MC6809MaterializeSpills rewrites an
; EXTRACT_LO_i16 / EXTRACT_HI_i16 source from a SPILL_D* slot to a byte
; staging register (AB or AA), the operand class on the EXTRACT pseudo
; (ADc, 16-bit) no longer matches the new operand value (8-bit byte).
; -verify-machineinstrs flagged this as "Illegal physical register for
; instruction: $ab is not a ADc register".
;
; Fix: when the destination of the EXTRACT happens to be the same staging
; byte (the typical case given the dst-class constraint that pins dst to
; ABc for LO / AAc for HI), the EXTRACT becomes a self-copy and is queued
; for erase via the existing ToErase vector.
;
; This test exercises the pattern by truncating two distinct i16 values
; that get pushed into SPILL_D* slots, then extracting their low bytes —
; the regalloc decision to spill them is forced by the high register
; pressure in the surrounding loop.

define void @bug267(ptr %dst, ptr %src, i16 %a, i16 %b, i16 %c, i16 %d) {
; CHECK-LABEL: bug267:
entry:
  br label %loop

loop:
  %i = phi i16 [ 0, %entry ], [ %i.next, %loop ]
  ; Force the regalloc to spill %a..%d into SPILL_D*.
  %ax = add i16 %a, %i
  %bx = add i16 %b, %i
  %cx = add i16 %c, %i
  %dx = add i16 %d, %i
  %ax8 = trunc i16 %ax to i8
  %bx8 = trunc i16 %bx to i8
  %cx8 = trunc i16 %cx to i8
  %dx8 = trunc i16 %dx to i8
  %sum1 = add i8 %ax8, %bx8
  %sum2 = add i8 %cx8, %dx8
  %sum  = add i8 %sum1, %sum2
  store i8 %sum, ptr %dst
  %i.next = add i16 %i, 1
  %done = icmp eq i16 %i.next, 16
  br i1 %done, label %exit, label %loop

exit:
  ret void
}
