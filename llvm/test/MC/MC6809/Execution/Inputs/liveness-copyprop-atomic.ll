; Bug #370: end-to-end VALUE check of the spill / frame-addressing / copy-prop
; machinery touched by Bug #357 (TFR-as-copy + late MachineCopyPropagation) and
; Bug #363/#366 (index-register redundant-reload elimination in
; MC6809PostRASpillOpt). Those passes have precise STATIC regression tests
; (bug357_tfr_copyprop.mir, bug366-stack-aliasing-store.mir); this test adds the
; runtime-value layer those static checks cannot provide — it confirms that
; copy-propagation on the ATOMIC registers (IX/IY/SU/SS) and the index reload
; elimination remain SOUND (produce the correct bytes), which is the property a
; future change to those passes is most likely to break silently.
;
; The accumulator family is deliberately NOT a target here: its #357/#366
; miscompiles are whole-program-cumulative and do not reproduce in a small
; value-checkable function (verified empirically against regressed compilers);
; the atomic registers are sound by construction, so a correct value across all
; opt levels is the meaningful invariant.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; On-stack array, filled from the args and walked through a pointer that is
; repeatedly re-materialised from the frame base (SU/SS) into an index register
; — exactly the `tfr s,y` / `ldy N,u` reload shape the #357/#363 passes act on.
; GISel performs no mem2reg, so the alloca stays on the frame and the index
; reloads are real.
define i16 @frame_sum(i16 %a, i16 %b, i16 %c) {
entry:
  %arr = alloca [6 x i16], align 1
  %p0 = getelementptr [6 x i16], ptr %arr, i16 0, i16 0
  %p1 = getelementptr [6 x i16], ptr %arr, i16 0, i16 1
  %p2 = getelementptr [6 x i16], ptr %arr, i16 0, i16 2
  %p3 = getelementptr [6 x i16], ptr %arr, i16 0, i16 3
  %p4 = getelementptr [6 x i16], ptr %arr, i16 0, i16 4
  %p5 = getelementptr [6 x i16], ptr %arr, i16 0, i16 5
  store i16 %a, ptr %p0, align 1
  store i16 %b, ptr %p1, align 1
  store i16 %c, ptr %p2, align 1
  %ab = add i16 %a, %b
  store i16 %ab, ptr %p3, align 1
  %bc = add i16 %b, %c
  store i16 %bc, ptr %p4, align 1
  %xr = xor i16 %a, %c
  store i16 %xr, ptr %p5, align 1
  br label %loop

loop:
  %i = phi i16 [ 0, %entry ], [ %inc, %loop ]
  %acc = phi i16 [ 0, %entry ], [ %nacc, %loop ]
  %q = phi ptr [ %p0, %entry ], [ %qn, %loop ]
  %v = load i16, ptr %q, align 1
  %s = add i16 %acc, %v
  %sh = shl i16 %s, 1
  %nacc = xor i16 %s, %sh
  %qn = getelementptr i16, ptr %q, i16 1
  %inc = add nuw nsw i16 %i, 1
  %done = icmp eq i16 %inc, 6
  br i1 %done, label %end, label %loop

end:
  ret i16 %nacc
}

; Pointer round-trip: each iteration copies the walk pointer through a temporary
; (the atomic index/stack TFRs MachineCopyPropagation collapses) and sums the
; loaded words. Correctness depends on those collapses being value-preserving.
define i16 @ptr_roundtrip(ptr %base, i16 %n) {
entry:
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %body, label %done

done:
  %res = phi i16 [ 0, %entry ], [ %add, %body ]
  ret i16 %res

body:
  %q = phi ptr [ %qn, %body ], [ %base, %entry ]
  %i = phi i16 [ %inc, %body ], [ 0, %entry ]
  %t = phi i16 [ %add, %body ], [ 0, %entry ]
  %v = load i16, ptr %q, align 1
  %add = add i16 %v, %t
  %qn = getelementptr i8, ptr %q, i16 2
  %inc = add nuw nsw i16 %i, 1
  %ec = icmp eq i16 %inc, %n
  br i1 %ec, label %done, label %body
}
