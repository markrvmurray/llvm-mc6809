; Hand-authored: after BranchRelaxation has widened a conditional branch that
; cannot reach its target into `b!cc X ; lbra L ; X:`, the pair folds back into
; the long conditional branch the 6809 has -- one byte shorter, and faster
; when the far target is the likely one -- so always when optimising for size,
; and otherwise only when the far target is taken at least half the time.
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s

; A loop whose body is too big for a short back edge.
; CHECK-LABEL: farloop:
; CHECK: lbne .LBB0_1
; CHECK-NOT: lbra

; A far `if` body with no reason to think it unlikely.
; CHECK-LABEL: farif:
; CHECK: lbne .LBB1_2
; CHECK-NOT: lbra

target triple = "mc6809-unknown-unknown"

@port = external global [16 x i8], align 1

define dso_local void @farloop(i16 noundef %n) local_unnamed_addr #0 {
entry:
  br label %do.body

do.body:                                          ; preds = %do.body, %entry
  %n.addr.0 = phi i16 [ %n, %entry ], [ %dec, %do.body ]
  store volatile i8 1, ptr @port, align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 1), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 2), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 3), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 4), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 5), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 6), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 7), align 1, !tbaa !9
  store volatile i8 1, ptr getelementptr inbounds nuw (i8, ptr @port, i16 8), align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 9), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 10), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 11), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 12), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 13), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 14), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 15), align 1, !tbaa !9
  store volatile i8 1, ptr @port, align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 1), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 2), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 3), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 4), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 5), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 6), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 7), align 1, !tbaa !9
  store volatile i8 1, ptr getelementptr inbounds nuw (i8, ptr @port, i16 8), align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 9), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 10), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 11), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 12), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 13), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 14), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 15), align 1, !tbaa !9
  %dec = add i16 %n.addr.0, -1
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %do.end, label %do.body, !llvm.loop !10

do.end:                                           ; preds = %do.body
  ret void
}

define dso_local range(i16 0, 2) i16 @farif(i16 noundef %n) local_unnamed_addr #0 {
entry:
  %cmp = icmp eq i16 %n, 7
  br i1 %cmp, label %if.then, label %return

if.then:                                          ; preds = %entry
  store volatile i8 1, ptr @port, align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 1), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 2), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 3), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 4), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 5), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 6), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 7), align 1, !tbaa !9
  store volatile i8 1, ptr getelementptr inbounds nuw (i8, ptr @port, i16 8), align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 9), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 10), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 11), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 12), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 13), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 14), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 15), align 1, !tbaa !9
  store volatile i8 1, ptr @port, align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 1), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 2), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 3), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 4), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 5), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 6), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 7), align 1, !tbaa !9
  store volatile i8 1, ptr getelementptr inbounds nuw (i8, ptr @port, i16 8), align 1, !tbaa !9
  store volatile i8 2, ptr getelementptr inbounds nuw (i8, ptr @port, i16 9), align 1, !tbaa !9
  store volatile i8 3, ptr getelementptr inbounds nuw (i8, ptr @port, i16 10), align 1, !tbaa !9
  store volatile i8 4, ptr getelementptr inbounds nuw (i8, ptr @port, i16 11), align 1, !tbaa !9
  store volatile i8 5, ptr getelementptr inbounds nuw (i8, ptr @port, i16 12), align 1, !tbaa !9
  store volatile i8 6, ptr getelementptr inbounds nuw (i8, ptr @port, i16 13), align 1, !tbaa !9
  store volatile i8 7, ptr getelementptr inbounds nuw (i8, ptr @port, i16 14), align 1, !tbaa !9
  store volatile i8 8, ptr getelementptr inbounds nuw (i8, ptr @port, i16 15), align 1, !tbaa !9
  br label %return

return:                                           ; preds = %entry, %if.then
  %retval.0 = phi i16 [ 1, %if.then ], [ 0, %entry ]
  ret i16 %retval.0
}

attributes #0 = { nofree norecurse nosync nounwind memory(readwrite, argmem: none, target_mem: none) "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0, !1, !2}
!llvm.errno.tbaa = !{!4}

!0 = !{i32 8, !"PIC Level", i32 2}
!1 = !{i32 7, !"PIE Level", i32 2}
!2 = !{i32 7, !"frame-pointer", i32 2}
!4 = !{!5, !6, i64 0}
!5 = !{!"__libc_errno", !6, i64 0}
!6 = !{!"int", !7, i64 0}
!7 = !{!"omnipotent char", !8, i64 0}
!8 = !{!"Simple C/C++ TBAA"}
!9 = !{!7, !7, i64 0}
!10 = distinct !{!10, !11}
!11 = !{!"llvm.loop.mustprogress"}
!12 = !{!"branch_weights", !"expected", i32 1, i32 2000}
