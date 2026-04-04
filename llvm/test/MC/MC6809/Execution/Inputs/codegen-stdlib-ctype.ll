; ModuleID = '/tmp/stdlib_funcs.c'
source_filename = "/tmp/stdlib_funcs.c"
target datalayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, -32768) i16 @test_abs(i16 noundef %i) local_unnamed_addr #0 {
entry:
  %cond = tail call i16 @llvm.abs.i16(i16 %i, i1 true)
  ret i16 %cond
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

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(argmem: write)
define dso_local void @test_div(ptr noundef writeonly captures(none) initializes((0, 4)) %result, i16 noundef %numer, i16 noundef %denom) local_unnamed_addr #2 {
entry:
  %div = sdiv i16 %numer, %denom
  store i16 %div, ptr %result, align 1, !tbaa !3
  %mul = mul nsw i16 %div, %denom
  %sub = sub nsw i16 %numer, %mul
  %arrayidx2 = getelementptr inbounds nuw i8, ptr %result, i16 2
  store i16 %sub, ptr %arrayidx2, align 1, !tbaa !3
  ret void
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

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isdigit(i16 noundef %c) local_unnamed_addr #0 {
entry:
  %cmp = icmp sgt i16 %c, 47
  br i1 %cmp, label %land.rhs, label %land.end

land.rhs:                                         ; preds = %entry
  %cmp1 = icmp samesign ult i16 %c, 58
  %0 = zext i1 %cmp1 to i16
  br label %land.end

land.end:                                         ; preds = %land.rhs, %entry
  %cond = phi i16 [ 0, %entry ], [ %0, %land.rhs ]
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isalpha(i16 noundef %c) local_unnamed_addr #0 {
entry:
  %0 = add i16 %c, -65
  %or.cond = icmp ult i16 %0, 26
  br i1 %or.cond, label %lor.end, label %lor.rhs

lor.rhs:                                          ; preds = %entry
  %cmp2 = icmp sgt i16 %c, 96
  br i1 %cmp2, label %land.rhs, label %lor.end

land.rhs:                                         ; preds = %lor.rhs
  %cmp3 = icmp samesign ult i16 %c, 123
  %1 = zext i1 %cmp3 to i16
  br label %lor.end

lor.end:                                          ; preds = %lor.rhs, %land.rhs, %entry
  %cond = phi i16 [ 1, %entry ], [ 0, %lor.rhs ], [ %1, %land.rhs ]
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isupper(i16 noundef %c) local_unnamed_addr #0 {
entry:
  %cmp = icmp sgt i16 %c, 64
  br i1 %cmp, label %land.rhs, label %land.end

land.rhs:                                         ; preds = %entry
  %cmp1 = icmp samesign ult i16 %c, 91
  %0 = zext i1 %cmp1 to i16
  br label %land.end

land.end:                                         ; preds = %land.rhs, %entry
  %cond = phi i16 [ 0, %entry ], [ %0, %land.rhs ]
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_islower(i16 noundef %c) local_unnamed_addr #0 {
entry:
  %cmp = icmp sgt i16 %c, 96
  br i1 %cmp, label %land.rhs, label %land.end

land.rhs:                                         ; preds = %entry
  %cmp1 = icmp samesign ult i16 %c, 123
  %0 = zext i1 %cmp1 to i16
  br label %land.end

land.end:                                         ; preds = %land.rhs, %entry
  %cond = phi i16 [ 0, %entry ], [ %0, %land.rhs ]
  ret i16 %cond
}

; Function Attrs: nocallback nofree nosync nounwind speculatable willreturn memory(none)
declare i16 @llvm.abs.i16(i16, i1 immarg) #3

attributes #0 = { mustprogress nofree norecurse nosync nounwind willreturn memory(none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree norecurse nosync nounwind memory(argmem: read) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #2 = { mustprogress nofree norecurse nosync nounwind willreturn memory(argmem: write) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #3 = { nocallback nofree nosync nounwind speculatable willreturn memory(none) }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}
!llvm.errno.tbaa = !{!3}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"frame-pointer", i32 2}
!2 = !{!"clang version 23.0.0git (git@github.com:markrvmurray/llvm-mc6809.git 22568e25f82cbf8b2bd711545c71ad9253f97c64)"}
!3 = !{!4, !4, i64 0}
!4 = !{!"int", !5, i64 0}
!5 = !{!"omnipotent char", !6, i64 0}
!6 = !{!"Simple C/C++ TBAA"}
!7 = !{!5, !5, i64 0}
!8 = distinct !{!8, !9}
!9 = !{!"llvm.loop.mustprogress"}
