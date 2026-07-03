; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -mattr=+static-stack -O2 %s -o - | FileCheck %s --check-prefix=SS
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s --check-prefix=DYN

; Static-stack allocation for non-reentrant functions (the +static-stack
; feature). A function carrying the "nonreentrant" attribute lays its whole
; local frame — including i32 spill slots — in a per-function static global
; addressed by extended / PC-relative mode instead of a dynamic U-relative
; frame. i32 is HD6309-only, so these run with -mcpu=hd6309.
;
; The DYN run (feature off) is the contrast: the identical IR keeps its
; dynamic U-relative frame and references no static-stack global. This is
; what guards against the feature leaking into the default configuration.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; A non-reentrant leaf: no calls, so the static frame makes the U frame
; pointer redundant. The prologue drops "pshs u" / "tfr s,u" entirely and U is
; free for allocation; the i32 accumulate reads the frame with Add_i32_Sym
; (addw/adcd against the static global).
define dso_local i32 @ss_leaf(i32 noundef %seed, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_leaf:
; SS-NOT:     tfr{{.*}}s,u
; SS:         .Lss_leaf_sstk
; SS:         addw{{.*}}.Lss_leaf_sstk
; SS:         adcd{{.*}}.Lss_leaf_sstk
;
; DYN-LABEL: ss_leaf:
; The dynamic frame is S-relative with no frame pointer since the stock
; frame-index spill migration (the old `tfr s,u` here was a side effect of
; the retired SPILL_Q slots forcing U setup).
; DYN:         leas
; DYN-NOT:     _sstk
entry:
  %buf = alloca [4 x i32], align 1
  store i32 %seed, ptr %buf, align 1
  %add.1 = add nsw i32 %seed, 3
  %gep.1 = getelementptr inbounds nuw i8, ptr %buf, i16 4
  store i32 %add.1, ptr %gep.1, align 1
  %add.2 = add nsw i32 %seed, 6
  %gep.2 = getelementptr inbounds nuw i8, ptr %buf, i16 8
  store i32 %add.2, ptr %gep.2, align 1
  %add.3 = add nsw i32 %seed, 9
  %gep.3 = getelementptr inbounds nuw i8, ptr %buf, i16 12
  store i32 %add.3, ptr %gep.3, align 1
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %body, label %done

done:                                             ; preds = %body, %entry
  %acc.lcssa = phi i32 [ 0, %entry ], [ %add8, %body ]
  ret i32 %acc.lcssa

body:                                             ; preds = %entry, %body
  %i = phi i16 [ %inc, %body ], [ 0, %entry ]
  %acc = phi i32 [ %add8, %body ], [ 0, %entry ]
  %and = and i16 %i, 3
  %idx = getelementptr inbounds nuw [4 x i8], ptr %buf, i16 %and
  %ld = load i32, ptr %idx, align 1
  %add8 = add nsw i32 %ld, %acc
  %inc = add nuw nsw i16 %i, 1
  %ec = icmp eq i16 %inc, %n
  br i1 %ec, label %done, label %body
}

; A non-reentrant function that makes a call. The return-address push shifts S,
; so S-relative access to the remaining fixed arguments would be unstable — the
; frame pointer is retained (tfr s,u) even though the locals still live in the
; static frame.
define dso_local i32 @ss_call(i32 noundef %seed, i16 noundef %n) local_unnamed_addr #0 {
; SS-LABEL: ss_call:
; SS:         tfr{{.*}}s,u
; SS:         .Lss_call_sstk
;
; DYN-LABEL: ss_call:
; DYN:         tfr{{.*}}s,u
; DYN-NOT:     _sstk
entry:
  %buf = alloca [4 x i32], align 1
  store i32 %seed, ptr %buf, align 1
  %add.1 = add nsw i32 %seed, 3
  %gep.1 = getelementptr inbounds nuw i8, ptr %buf, i16 4
  store i32 %add.1, ptr %gep.1, align 1
  %add.2 = add nsw i32 %seed, 6
  %gep.2 = getelementptr inbounds nuw i8, ptr %buf, i16 8
  store i32 %add.2, ptr %gep.2, align 1
  %add.3 = add nsw i32 %seed, 9
  %gep.3 = getelementptr inbounds nuw i8, ptr %buf, i16 12
  store i32 %add.3, ptr %gep.3, align 1
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %body, label %done

done:                                             ; preds = %body, %entry
  %acc.lcssa = phi i32 [ 0, %entry ], [ %add8, %body ]
  ret i32 %acc.lcssa

body:                                             ; preds = %entry, %body
  %i = phi i16 [ %inc, %body ], [ 0, %entry ]
  %acc = phi i32 [ %add8, %body ], [ 0, %entry ]
  %and = and i16 %i, 3
  %idx = getelementptr inbounds nuw [4 x i8], ptr %buf, i16 %and
  %ld = load i32, ptr %idx, align 1
  %call = tail call i32 @ext_helper(i32 noundef %ld)
  %add8 = add nsw i32 %call, %acc
  %inc = add nuw nsw i16 %i, 1
  %ec = icmp eq i16 %inc, %n
  br i1 %ec, label %done, label %body
}

declare i32 @ext_helper(i32)

attributes #0 = { nounwind "nonreentrant" "target-cpu"="hd6309" }
