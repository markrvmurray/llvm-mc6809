; RUN: llc -mtriple=mc6809 -O2 < %s | FileCheck %s
;
; Bug #350: a 16-bit signed-add-with-overflow whose second addend is a
; runtime negation (sub nsw i16 0, %k) splits into a low-byte ADD and a
; high-byte carry-in ADC. The SetOverflowUse reg pseudo was missing from
; MaterializeSpills' needsSecondAccSpillSkip list, so both distinct ACC
; spill operands of the high byte were loaded into $ab — the second LDB
; clobbering the first — and the ADC computed "src2 + src2" instead of
; "src1 + src2". For exp=128,k=2 the result became 0xFF7E (-130) rather
; than 126, which then took the `s <= 0` branch. Manifested as a
; complex-divide miscompile (__divsc3 / __divdc3 returning +/-Inf).
;
; With the fix the high-byte ADC's two operands stay separate. Under the
; original SPILL_* materialisation this showed as an `adcb N,u` slot read
; (emit6809RegByteFromMem Path(a) via the needsSecondAccSpillSkip list);
; under stock frame-index spilling the same separation shows as a
; cross-half push sequence (`pshs a; adcb ,s+` or the mirrored halves) —
; the RHS high byte lives in the other accumulator half and is consumed
; from the S stack, never a second same-register load clobbering the
; first operand. Which half carries which operand is the allocator's
; choice now that the byte pseudos take either page-1 half.

; CHECK-LABEL: t1:
; The high-byte add-with-carry consumes its RHS from the stack (pushed
; from the other half), proving the two operands did not collapse.
; CHECK: pshs {{[ab]}}
; CHECK-NEXT: adc{{[ab]}} ,s+

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"

define dso_local i16 @t1(i16 noundef %exp, i16 noundef %k) {
entry:
  %sub = sub nsw i16 0, %k
  %0 = tail call { i16, i1 } @llvm.sadd.with.overflow.i16(i16 %exp, i16 %sub)
  %1 = extractvalue { i16, i1 } %0, 1
  br i1 %1, label %cleanup, label %if.end
if.end:
  %2 = extractvalue { i16, i1 } %0, 0
  %cmp = icmp slt i16 %2, 1
  %. = select i1 %cmp, i16 -1, i16 %2
  br label %cleanup
cleanup:
  %retval.0 = phi i16 [ -999, %entry ], [ %., %if.end ]
  ret i16 %retval.0
}

declare { i16, i1 } @llvm.sadd.with.overflow.i16(i16, i16)
