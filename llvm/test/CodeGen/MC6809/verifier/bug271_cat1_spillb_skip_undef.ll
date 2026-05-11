; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O1 -verify-machineinstrs < %s -o /dev/null
;
; Bug #271 cat-1 sentinel — SPILL_* skip Undef subset.
;
; MC6809MaterializeSpills's `needsSecondAccSpillSkip` path leaves the
; SECOND-and-later distinct ACC spill operand on Add/Sub-SetCarry-family
; _Reg byte pseudos unmaterialised — ExpandPostRAPseudo handles them
; via a U-relative spill path that combines the load with the
; arithmetic (`adcb [slot,$u]`). But the SPILL_* phys-reg operand
; survives post-MaterializeSpills with no visible def (the def became
; a Store_i*_Mem to a frame slot, not a phys-reg def), so the verifier
; flags every such second SPILL_* USE operand as "Using an undefined
; physical register".
;
; Fix: when MaterializeSpills decides to SKIP a SPILL_* USE (either
; because it's the 2nd+ unique ACC spill or because the first spill
; collides with a same-MI physical operand), mark that operand
; RegState::Undef. The verifier then skips its liveness check;
; ExpandPostRAPseudo still reads the SPILL_* identity to compute the
; U-relative offset, so codegen is byte-identical (libc.a at
; Og-hd6309-mame: 2687608 bytes before and after).
;
; Reproducer: chain enough byte-level carry-using arithmetic to force
; regalloc to spill multiple ACC values, triggering the second-acc
; spill skip in the AddSetCarryUse_i8_Reg expansion path. The pattern
; is the same shape that drives `__dorand48`'s seed multiplication
; in picolibc rand48.c at -Og.

target triple = "mc6809-unknown-unknown"

@a = external global [16 x i8], align 1

; 8-byte chained-addition: c[i] = a[i] + a[i+8] across all 8 bytes,
; with the carries threaded through. The high-pressure regalloc
; spills multiple intermediate bytes into SPILL_B*, and the inner
; AddSetCarryUse pseudos hit the second-acc skip path.
define void @bug271_cat1_spillb_eightbyte_add(ptr %dst) nounwind {
entry:
  %p0 = getelementptr [16 x i8], ptr @a, i16 0, i16 0
  %p1 = getelementptr [16 x i8], ptr @a, i16 0, i16 1
  %p2 = getelementptr [16 x i8], ptr @a, i16 0, i16 2
  %p3 = getelementptr [16 x i8], ptr @a, i16 0, i16 3
  %p4 = getelementptr [16 x i8], ptr @a, i16 0, i16 4
  %p5 = getelementptr [16 x i8], ptr @a, i16 0, i16 5
  %p6 = getelementptr [16 x i8], ptr @a, i16 0, i16 6
  %p7 = getelementptr [16 x i8], ptr @a, i16 0, i16 7
  %v0 = load i8, ptr %p0, align 1
  %v1 = load i8, ptr %p1, align 1
  %v2 = load i8, ptr %p2, align 1
  %v3 = load i8, ptr %p3, align 1
  %v4 = load i8, ptr %p4, align 1
  %v5 = load i8, ptr %p5, align 1
  %v6 = load i8, ptr %p6, align 1
  %v7 = load i8, ptr %p7, align 1
  %x0 = zext i8 %v0 to i64
  %x1 = zext i8 %v1 to i64
  %x2 = zext i8 %v2 to i64
  %x3 = zext i8 %v3 to i64
  %x4 = zext i8 %v4 to i64
  %x5 = zext i8 %v5 to i64
  %x6 = zext i8 %v6 to i64
  %x7 = zext i8 %v7 to i64
  %s0 = shl i64 %x0, 0
  %s1 = shl i64 %x1, 8
  %s2 = shl i64 %x2, 16
  %s3 = shl i64 %x3, 24
  %s4 = shl i64 %x4, 32
  %s5 = shl i64 %x5, 40
  %s6 = shl i64 %x6, 48
  %s7 = shl i64 %x7, 56
  %a01 = or i64 %s0, %s1
  %a23 = or i64 %s2, %s3
  %a45 = or i64 %s4, %s5
  %a67 = or i64 %s6, %s7
  %a0123 = or i64 %a01, %a23
  %a4567 = or i64 %a45, %a67
  %a_lhs = or i64 %a0123, %a4567

  %q0 = getelementptr [16 x i8], ptr @a, i16 0, i16 8
  %q1 = getelementptr [16 x i8], ptr @a, i16 0, i16 9
  %q2 = getelementptr [16 x i8], ptr @a, i16 0, i16 10
  %q3 = getelementptr [16 x i8], ptr @a, i16 0, i16 11
  %q4 = getelementptr [16 x i8], ptr @a, i16 0, i16 12
  %q5 = getelementptr [16 x i8], ptr @a, i16 0, i16 13
  %q6 = getelementptr [16 x i8], ptr @a, i16 0, i16 14
  %q7 = getelementptr [16 x i8], ptr @a, i16 0, i16 15
  %w0 = load i8, ptr %q0, align 1
  %w1 = load i8, ptr %q1, align 1
  %w2 = load i8, ptr %q2, align 1
  %w3 = load i8, ptr %q3, align 1
  %w4 = load i8, ptr %q4, align 1
  %w5 = load i8, ptr %q5, align 1
  %w6 = load i8, ptr %q6, align 1
  %w7 = load i8, ptr %q7, align 1
  %y0 = zext i8 %w0 to i64
  %y1 = zext i8 %w1 to i64
  %y2 = zext i8 %w2 to i64
  %y3 = zext i8 %w3 to i64
  %y4 = zext i8 %w4 to i64
  %y5 = zext i8 %w5 to i64
  %y6 = zext i8 %w6 to i64
  %y7 = zext i8 %w7 to i64
  %t0 = shl i64 %y0, 0
  %t1 = shl i64 %y1, 8
  %t2 = shl i64 %y2, 16
  %t3 = shl i64 %y3, 24
  %t4 = shl i64 %y4, 32
  %t5 = shl i64 %y5, 40
  %t6 = shl i64 %y6, 48
  %t7 = shl i64 %y7, 56
  %b01 = or i64 %t0, %t1
  %b23 = or i64 %t2, %t3
  %b45 = or i64 %t4, %t5
  %b67 = or i64 %t6, %t7
  %b0123 = or i64 %b01, %b23
  %b4567 = or i64 %b45, %b67
  %b_rhs = or i64 %b0123, %b4567

  %sum = add i64 %a_lhs, %b_rhs
  store i64 %sum, ptr %dst, align 1
  ret void
}
