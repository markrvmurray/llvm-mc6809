; ModuleID = '/tmp/picolibc-all.c'
source_filename = "/tmp/picolibc-all.c"
target datalayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_memcpy(ptr noundef returned writeonly captures(ret: address, provenance) %dst, ptr noundef readonly captures(none) %src, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not3 = icmp eq i16 %n, 0
  br i1 %tobool.not3, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %s.06 = phi ptr [ %incdec.ptr, %while.body ], [ %src, %entry ]
  %d.05 = phi ptr [ %incdec.ptr1, %while.body ], [ %dst, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %dec = add i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.06, i16 1
  %0 = load i8, ptr %s.06, align 1, !tbaa !6
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.05, i16 1
  store i8 %0, ptr %d.05, align 1, !tbaa !6
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !7

while.end:                                        ; preds = %while.body, %entry
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: write)
define dso_local noundef ptr @test_memset(ptr noundef returned writeonly captures(ret: address, provenance) %dst, i16 noundef %c, i16 noundef %n) local_unnamed_addr #1 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body.lr.ph

while.body.lr.ph:                                 ; preds = %entry
  %conv = trunc i16 %c to i8
  br label %while.body

while.body:                                       ; preds = %while.body.lr.ph, %while.body
  %d.04 = phi ptr [ %dst, %while.body.lr.ph ], [ %incdec.ptr, %while.body ]
  %n.addr.03 = phi i16 [ %n, %while.body.lr.ph ], [ %dec, %while.body ]
  %dec = add i16 %n.addr.03, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %d.04, i16 1
  store i8 %conv, ptr %d.04, align 1, !tbaa !6
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !9

while.end:                                        ; preds = %while.body, %entry
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_memmove(ptr noundef returned writeonly captures(address, ret: address, provenance) %dst, ptr noundef readonly captures(address) %src, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %cmp = icmp ult ptr %dst, %src
  %tobool.not25 = icmp eq i16 %n, 0
  br i1 %cmp, label %while.cond.preheader, label %if.else

while.cond.preheader:                             ; preds = %entry
  br i1 %tobool.not25, label %if.end, label %while.body

while.body:                                       ; preds = %while.cond.preheader, %while.body
  %s.028 = phi ptr [ %incdec.ptr, %while.body ], [ %src, %while.cond.preheader ]
  %d.027 = phi ptr [ %incdec.ptr1, %while.body ], [ %dst, %while.cond.preheader ]
  %n.addr.026 = phi i16 [ %dec, %while.body ], [ %n, %while.cond.preheader ]
  %dec = add i16 %n.addr.026, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.028, i16 1
  %0 = load i8, ptr %s.028, align 1, !tbaa !6
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.027, i16 1
  store i8 %0, ptr %d.027, align 1, !tbaa !6
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %if.end, label %while.body, !llvm.loop !10

if.else:                                          ; preds = %entry
  br i1 %tobool.not25, label %if.end, label %while.body6.preheader

while.body6.preheader:                            ; preds = %if.else
  %add.ptr2 = getelementptr inbounds nuw i8, ptr %src, i16 %n
  %add.ptr = getelementptr inbounds nuw i8, ptr %dst, i16 %n
  br label %while.body6

while.body6:                                      ; preds = %while.body6.preheader, %while.body6
  %s.124 = phi ptr [ %incdec.ptr7, %while.body6 ], [ %add.ptr2, %while.body6.preheader ]
  %d.123 = phi ptr [ %incdec.ptr8, %while.body6 ], [ %add.ptr, %while.body6.preheader ]
  %n.addr.122 = phi i16 [ %dec4, %while.body6 ], [ %n, %while.body6.preheader ]
  %dec4 = add i16 %n.addr.122, -1
  %incdec.ptr7 = getelementptr inbounds i8, ptr %s.124, i16 -1
  %1 = load i8, ptr %incdec.ptr7, align 1, !tbaa !6
  %incdec.ptr8 = getelementptr inbounds i8, ptr %d.123, i16 -1
  store i8 %1, ptr %incdec.ptr8, align 1, !tbaa !6
  %tobool5.not = icmp eq i16 %dec4, 0
  br i1 %tobool5.not, label %if.end, label %while.body6, !llvm.loop !11

if.end:                                           ; preds = %while.body6, %while.body, %if.else, %while.cond.preheader
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local range(i16 -255, 256) i16 @test_memcmp(ptr noundef readonly captures(none) %s1, ptr noundef readonly captures(none) %s2, i16 noundef %n) local_unnamed_addr #2 {
entry:
  %tobool.not13 = icmp eq i16 %n, 0
  br i1 %tobool.not13, label %cleanup, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %dec16.in = phi i16 [ %dec16, %if.end ], [ %n, %entry ]
  %p2.015 = phi ptr [ %incdec.ptr5, %if.end ], [ %s2, %entry ]
  %p1.014 = phi ptr [ %incdec.ptr, %if.end ], [ %s1, %entry ]
  %0 = load i8, ptr %p1.014, align 1, !tbaa !6
  %1 = load i8, ptr %p2.015, align 1, !tbaa !6
  %cmp.not = icmp eq i8 %0, %1
  br i1 %cmp.not, label %if.end, label %if.then

if.then:                                          ; preds = %while.body
  %conv1 = zext i8 %1 to i16
  %conv = zext i8 %0 to i16
  %sub = sub nsw i16 %conv, %conv1
  br label %cleanup

if.end:                                           ; preds = %while.body
  %dec16 = add i16 %dec16.in, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %p1.014, i16 1
  %incdec.ptr5 = getelementptr inbounds nuw i8, ptr %p2.015, i16 1
  %tobool.not = icmp eq i16 %dec16, 0
  br i1 %tobool.not, label %cleanup, label %while.body, !llvm.loop !12

cleanup:                                          ; preds = %if.end, %entry, %if.then
  %retval.0 = phi i16 [ %sub, %if.then ], [ 0, %entry ], [ 0, %if.end ]
  ret i16 %retval.0
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_strcpy(ptr noundef returned writeonly captures(ret: address, provenance) %dst, ptr noundef readonly captures(none) %src) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.cond, %entry
  %src.addr.0 = phi ptr [ %src, %entry ], [ %incdec.ptr, %while.cond ]
  %d.0 = phi ptr [ %dst, %entry ], [ %incdec.ptr1, %while.cond ]
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %src.addr.0, i16 1
  %0 = load i8, ptr %src.addr.0, align 1, !tbaa !6
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.0, i16 1
  store i8 %0, ptr %d.0, align 1, !tbaa !6
  %tobool.not = icmp eq i8 %0, 0
  br i1 %tobool.not, label %while.end, label %while.cond, !llvm.loop !13

while.end:                                        ; preds = %while.cond
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef ptr @test_strchr(ptr noundef readonly captures(ret: address, provenance) %s, i16 noundef %c) local_unnamed_addr #2 {
entry:
  %0 = trunc i16 %c to i8
  %1 = load i8, ptr %s, align 1, !tbaa !6
  %tobool.not7 = icmp eq i8 %1, 0
  br i1 %tobool.not7, label %cleanup, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %2 = phi i8 [ %3, %if.end ], [ %1, %entry ]
  %s.addr.08 = phi ptr [ %incdec.ptr, %if.end ], [ %s, %entry ]
  %cmp = icmp eq i8 %2, %0
  br i1 %cmp, label %cleanup, label %if.end

if.end:                                           ; preds = %while.body
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.08, i16 1
  %3 = load i8, ptr %incdec.ptr, align 1, !tbaa !6
  %tobool.not = icmp eq i8 %3, 0
  br i1 %tobool.not, label %cleanup, label %while.body, !llvm.loop !14

cleanup:                                          ; preds = %while.body, %if.end, %entry
  %retval.0 = phi ptr [ null, %entry ], [ null, %if.end ], [ %s.addr.08, %while.body ]
  ret ptr %retval.0
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local range(i16 -255, 256) i16 @test_strncmp(ptr noundef readonly captures(none) %s1, ptr noundef readonly captures(none) %s2, i16 noundef %n) local_unnamed_addr #2 {
entry:
  %tobool.not18 = icmp eq i16 %n, 0
  br i1 %tobool.not18, label %return, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %dec21.in = phi i16 [ %dec21, %if.end ], [ %n, %entry ]
  %s2.addr.020 = phi ptr [ %incdec.ptr10, %if.end ], [ %s2, %entry ]
  %s1.addr.019 = phi ptr [ %incdec.ptr, %if.end ], [ %s1, %entry ]
  %0 = load i8, ptr %s1.addr.019, align 1, !tbaa !6
  %1 = load i8, ptr %s2.addr.020, align 1, !tbaa !6
  %cmp.not = icmp eq i8 %0, %1
  br i1 %cmp.not, label %if.end, label %if.then

if.then:                                          ; preds = %while.body
  %conv = zext i8 %0 to i16
  %conv1 = zext i8 %1 to i16
  %sub = sub nsw i16 %conv, %conv1
  br label %return

if.end:                                           ; preds = %while.body
  %dec21 = add i16 %dec21.in, -1
  %cmp6 = icmp eq i8 %0, 0
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s1.addr.019, i16 1
  %incdec.ptr10 = getelementptr inbounds nuw i8, ptr %s2.addr.020, i16 1
  %tobool.not = icmp eq i16 %dec21, 0
  %or.cond = select i1 %cmp6, i1 true, i1 %tobool.not
  br i1 %or.cond, label %return, label %while.body, !llvm.loop !15

return:                                           ; preds = %if.end, %entry, %if.then
  %retval.0 = phi i16 [ %sub, %if.then ], [ 0, %entry ], [ 0, %if.end ]
  ret i16 %retval.0
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isspace(i16 noundef %c) local_unnamed_addr #3 {
entry:
  switch i16 %c, label %lor.rhs [
    i16 32, label %lor.end
    i16 13, label %lor.end
    i16 12, label %lor.end
    i16 10, label %lor.end
    i16 9, label %lor.end
  ]

lor.rhs:                                          ; preds = %entry
  %cmp8 = icmp eq i16 %c, 11
  %0 = zext i1 %cmp8 to i16
  br label %lor.end

lor.end:                                          ; preds = %entry, %entry, %entry, %entry, %entry, %lor.rhs
  %lor.ext = phi i16 [ 1, %entry ], [ %0, %lor.rhs ], [ 1, %entry ], [ 1, %entry ], [ 1, %entry ], [ 1, %entry ]
  ret i16 %lor.ext
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isprint(i16 noundef %c) local_unnamed_addr #3 {
entry:
  %cmp = icmp sgt i16 %c, 31
  br i1 %cmp, label %land.rhs, label %land.end

land.rhs:                                         ; preds = %entry
  %cmp1 = icmp samesign ult i16 %c, 127
  %0 = zext i1 %cmp1 to i16
  br label %land.end

land.end:                                         ; preds = %land.rhs, %entry
  %land.ext = phi i16 [ 0, %entry ], [ %0, %land.rhs ]
  ret i16 %land.ext
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isxdigit(i16 noundef %c) local_unnamed_addr #3 {
entry:
  %0 = add i16 %c, -48
  %or.cond = icmp ult i16 %0, 10
  %1 = add i16 %c, -97
  %or.cond7 = icmp ult i16 %1, 6
  %or.cond13 = or i1 %or.cond, %or.cond7
  br i1 %or.cond13, label %lor.end, label %lor.rhs

lor.rhs:                                          ; preds = %entry
  %cmp5 = icmp sgt i16 %c, 64
  br i1 %cmp5, label %land.rhs, label %lor.end

land.rhs:                                         ; preds = %lor.rhs
  %cmp6 = icmp samesign ult i16 %c, 71
  %2 = zext i1 %cmp6 to i16
  br label %lor.end

lor.end:                                          ; preds = %lor.rhs, %land.rhs, %entry
  %lor.ext = phi i16 [ %2, %land.rhs ], [ 1, %entry ], [ 0, %lor.rhs ]
  ret i16 %lor.ext
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local i16 @test_tolower(i16 noundef %c) local_unnamed_addr #3 {
entry:
  %0 = add i16 %c, -65
  %or.cond = icmp ult i16 %0, 26
  %add = or disjoint i16 %c, 32
  %cond = select i1 %or.cond, i16 %add, i16 %c
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local i16 @test_toupper(i16 noundef %c) local_unnamed_addr #3 {
entry:
  %0 = add i16 %c, -97
  %or.cond = icmp ult i16 %0, 26
  %sub = add nsw i16 %c, -32
  %cond = select i1 %or.cond, i16 %sub, i16 %c
  ret i16 %cond
}

attributes #0 = { nofree norecurse nosync nounwind memory(argmem: readwrite) "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree norecurse nosync nounwind memory(argmem: write) "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #2 = { nofree norecurse nosync nounwind memory(argmem: read) "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #3 = { mustprogress nofree norecurse nosync nounwind willreturn memory(none) "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}
!llvm.errno.tbaa = !{!2}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 23.0.0git (git@github.com:markrvmurray/llvm-mc6809.git 56561fa834fde327dbb1e4173043ab5800d27671)"}
!2 = !{!3, !3, i64 0}
!3 = !{!"int", !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
!6 = !{!4, !4, i64 0}
!7 = distinct !{!7, !8}
!8 = !{!"llvm.loop.mustprogress"}
!9 = distinct !{!9, !8}
!10 = distinct !{!10, !8}
!11 = distinct !{!11, !8}
!12 = distinct !{!12, !8}
!13 = distinct !{!13, !8}
!14 = distinct !{!14, !8}
!15 = distinct !{!15, !8}
