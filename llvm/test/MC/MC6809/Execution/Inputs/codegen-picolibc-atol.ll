; ModuleID = '/tmp/picolibc-atol.c'
source_filename = "/tmp/picolibc-atol.c"
target datalayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local i32 @test_atol(ptr noundef readonly captures(none) %s) local_unnamed_addr #0 {
entry:
  %0 = load i8, ptr %s, align 1, !tbaa !7
  %cmp = icmp eq i8 %0, 45
  br i1 %cmp, label %if.then, label %if.else

if.then:                                          ; preds = %entry
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s, i16 1
  br label %if.end7

if.else:                                          ; preds = %entry
  %cmp3 = icmp eq i8 %0, 43
  br i1 %cmp3, label %if.then5, label %if.end7

if.then5:                                         ; preds = %if.else
  %incdec.ptr6 = getelementptr inbounds nuw i8, ptr %s, i16 1
  br label %if.end7

if.end7:                                          ; preds = %if.else, %if.then5, %if.then
  %s.addr.0 = phi ptr [ %incdec.ptr, %if.then ], [ %incdec.ptr6, %if.then5 ], [ %s, %if.else ]
  %1 = load i8, ptr %s.addr.0, align 1, !tbaa !7
  %2 = add i8 %1, -48
  %or.cond29 = icmp ult i8 %2, 10
  br i1 %or.cond29, label %while.body, label %while.end

while.body:                                       ; preds = %if.end7, %while.body
  %3 = phi i8 [ %4, %while.body ], [ %1, %if.end7 ]
  %result.031 = phi i32 [ %add, %while.body ], [ 0, %if.end7 ]
  %s.addr.130 = phi ptr [ %incdec.ptr16, %while.body ], [ %s.addr.0, %if.end7 ]
  %conv8 = zext nneg i8 %3 to i16
  %mul = mul nsw i32 %result.031, 10
  %sub = add nsw i16 %conv8, -48
  %conv15 = zext nneg i16 %sub to i32
  %add = add nsw i32 %mul, %conv15
  %incdec.ptr16 = getelementptr inbounds nuw i8, ptr %s.addr.130, i16 1
  %4 = load i8, ptr %incdec.ptr16, align 1, !tbaa !7
  %5 = add i8 %4, -48
  %or.cond = icmp ult i8 %5, 10
  br i1 %or.cond, label %while.body, label %while.end, !llvm.loop !8

while.end:                                        ; preds = %while.body, %if.end7
  %result.0.lcssa = phi i32 [ 0, %if.end7 ], [ %add, %while.body ]
  %sub19 = sub nsw i32 0, %result.0.lcssa
  %cond = select i1 %cmp, i32 %sub19, i32 %result.0.lcssa
  ret i32 %cond
}

attributes #0 = { nofree norecurse nosync nounwind memory(argmem: read) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}
!llvm.errno.tbaa = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"frame-pointer", i32 2}
!2 = !{!"clang version 23.0.0git (git@github.com:markrvmurray/llvm-mc6809.git 56561fa834fde327dbb1e4173043ab5800d27671)"}
!3 = !{!4, !4, i64 0}
!4 = !{!"int", !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}
!7 = !{!5, !5, i64 0}
!8 = distinct !{!8, !9}
!9 = !{!"llvm.loop.mustprogress"}
