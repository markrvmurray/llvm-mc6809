; ModuleID = 'loop.c'
source_filename = "loop.c"
target datalayout = "e-p:16:8-S8-m:e-i1:8:8-i8:8:8-i16:8:8-i32:8:8-i64:8:8-f16:8:8-f32:8:8-f64:8:8-f128:8:8-a:0:8-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: argmemonly nofree norecurse nosync nounwind readonly
define dso_local i16 @loop(ptr nocapture noundef readonly %pa, i8 noundef signext %n) local_unnamed_addr #0 {
entry:
  %cmp6 = icmp sgt i8 %n, 0
  br i1 %cmp6, label %for.body.preheader, label %for.end

for.body.preheader:                               ; preds = %entry
  %wide.trip.count = zext i8 %n to i16
  br label %for.body

for.body:                                         ; preds = %for.body.preheader, %for.body
  %lsr.iv14 = phi i16 [ %wide.trip.count, %for.body.preheader ], [ %lsr.iv.next, %for.body ]
  %lsr.iv = phi ptr [ %pa, %for.body.preheader ], [ %uglygep13, %for.body ]
  %s.07 = phi i16 [ 0, %for.body.preheader ], [ %add, %for.body ]
  %0 = load i16, ptr %lsr.iv, align 1, !tbaa !3
  %add = add nsw i16 %0, %s.07
  %uglygep13 = getelementptr i8, ptr %lsr.iv, i16 2
  %lsr.iv.next = add nsw i16 %lsr.iv14, -1
  %exitcond.not = icmp eq i16 %lsr.iv.next, 0
  br i1 %exitcond.not, label %for.end, label %for.body, !llvm.loop !7

for.end:                                          ; preds = %for.body, %entry
  %s.0.lcssa = phi i16 [ 0, %entry ], [ %add, %for.body ]
  ret i16 %s.0.lcssa
}

attributes #0 = { argmemonly nofree norecurse nosync nounwind readonly "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"frame-pointer", i32 2}
!2 = !{!"clang version 15.0.0 (https://github.com/markrvmurray/llvm-mc6809.git 3fe96cc89c1380c464065b93a4a93c7ed9052f94)"}
!3 = !{!4, !4, i64 0}
!4 = !{!"int", !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}
!7 = distinct !{!7, !8}
!8 = !{!"llvm.loop.mustprogress"}
