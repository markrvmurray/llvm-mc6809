; ModuleID = '/tmp/atoi_funcs.c'
source_filename = "/tmp/atoi_funcs.c"
target datalayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, -32768) i16 @test_abs(i16 noundef %i) local_unnamed_addr #0 {
entry:
  %cond = tail call i16 @llvm.abs.i16(i16 %i, i1 true)
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local noundef zeroext i16 @test_or16(i16 noundef zeroext %a, i16 noundef zeroext %b) local_unnamed_addr #0 {
entry:
  %or = or i16 %b, %a
  ret i16 %or
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local noundef zeroext i16 @test_and16(i16 noundef zeroext %a, i16 noundef zeroext %b) local_unnamed_addr #0 {
entry:
  %and = and i16 %b, %a
  ret i16 %and
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local noundef zeroext i16 @test_xor16(i16 noundef zeroext %a, i16 noundef zeroext %b) local_unnamed_addr #0 {
entry:
  %xor = xor i16 %b, %a
  ret i16 %xor
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local i16 @test_atoi(ptr noundef readonly captures(none) %s) local_unnamed_addr #1 {
entry:
  %0 = load i8, ptr %s, align 1, !tbaa !7
  %1 = add i8 %0, -48
  %or.cond10 = icmp ult i8 %1, 10
  br i1 %or.cond10, label %while.body, label %while.end

while.body:                                       ; preds = %entry, %while.body
  %2 = phi i8 [ %3, %while.body ], [ %0, %entry ]
  %result.012 = phi i16 [ %add, %while.body ], [ 0, %entry ]
  %s.addr.011 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %mul = mul nsw i16 %result.012, 10
  %narrow = add nsw i8 %2, -48
  %sub = zext nneg i8 %narrow to i16
  %add = add nsw i16 %mul, %sub
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.011, i16 1
  %3 = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %4 = add i8 %3, -48
  %or.cond = icmp ult i8 %4, 10
  br i1 %or.cond, label %while.body, label %while.end, !llvm.loop !8

while.end:                                        ; preds = %while.body, %entry
  %result.0.lcssa = phi i16 [ 0, %entry ], [ %add, %while.body ]
  ret i16 %result.0.lcssa
}

; Function Attrs: nocallback nofree nosync nounwind speculatable willreturn memory(none)
declare i16 @llvm.abs.i16(i16, i1 immarg) #2

attributes #0 = { mustprogress nofree norecurse nosync nounwind willreturn memory(none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree norecurse nosync nounwind memory(argmem: read) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #2 = { nocallback nofree nosync nounwind speculatable willreturn memory(none) }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}
!llvm.errno.tbaa = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"frame-pointer", i32 2}
!2 = !{!"clang version 23.0.0git (git@github.com:markrvmurray/llvm-mc6809.git 3e60ce25fb917b696a2637436041bf6ffb6b41d9)"}
!3 = !{!4, !4, i64 0}
!4 = !{!"int", !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}
!7 = !{!5, !5, i64 0}
!8 = distinct !{!8, !9}
!9 = !{!"llvm.loop.mustprogress"}
