; ModuleID = '/tmp/picolibc-all.c'
source_filename = "/tmp/picolibc-all.c"
target datalayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

@test_strtok.next = internal unnamed_addr global ptr null, align 1
@rand_state = internal unnamed_addr global i16 1, align 1

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
  %0 = load i8, ptr %s.06, align 1, !tbaa !7
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.05, i16 1
  store i8 %0, ptr %d.05, align 1, !tbaa !7
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !8

while.end:                                        ; preds = %while.body, %entry
  ret ptr %dst
}

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite)
declare void @llvm.lifetime.start.p0(ptr captures(none)) #1

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite)
declare void @llvm.lifetime.end.p0(ptr captures(none)) #1

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: write)
define dso_local noundef ptr @test_memset(ptr noundef returned writeonly captures(ret: address, provenance) %dst, i16 noundef %c, i16 noundef %n) local_unnamed_addr #2 {
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
  store i8 %conv, ptr %d.04, align 1, !tbaa !7
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !10

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
  %0 = load i8, ptr %s.028, align 1, !tbaa !7
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.027, i16 1
  store i8 %0, ptr %d.027, align 1, !tbaa !7
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %if.end, label %while.body, !llvm.loop !11

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
  %1 = load i8, ptr %incdec.ptr7, align 1, !tbaa !7
  %incdec.ptr8 = getelementptr inbounds i8, ptr %d.123, i16 -1
  store i8 %1, ptr %incdec.ptr8, align 1, !tbaa !7
  %tobool5.not = icmp eq i16 %dec4, 0
  br i1 %tobool5.not, label %if.end, label %while.body6, !llvm.loop !12

if.end:                                           ; preds = %while.body6, %while.body, %if.else, %while.cond.preheader
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local range(i16 -255, 256) i16 @test_memcmp(ptr noundef readonly captures(none) %s1, ptr noundef readonly captures(none) %s2, i16 noundef %n) local_unnamed_addr #3 {
entry:
  %tobool.not13 = icmp eq i16 %n, 0
  br i1 %tobool.not13, label %cleanup, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %dec16.in = phi i16 [ %dec16, %if.end ], [ %n, %entry ]
  %p2.015 = phi ptr [ %incdec.ptr5, %if.end ], [ %s2, %entry ]
  %p1.014 = phi ptr [ %incdec.ptr, %if.end ], [ %s1, %entry ]
  %0 = load i8, ptr %p1.014, align 1, !tbaa !7
  %1 = load i8, ptr %p2.015, align 1, !tbaa !7
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
  br i1 %tobool.not, label %cleanup, label %while.body, !llvm.loop !13

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
  %0 = load i8, ptr %src.addr.0, align 1, !tbaa !7
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.0, i16 1
  store i8 %0, ptr %d.0, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %0, 0
  br i1 %tobool.not, label %while.end, label %while.cond, !llvm.loop !14

while.end:                                        ; preds = %while.cond
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef ptr @test_strchr(ptr noundef readonly captures(ret: address, provenance) %s, i16 noundef %c) local_unnamed_addr #3 {
entry:
  %0 = trunc i16 %c to i8
  %1 = load i8, ptr %s, align 1, !tbaa !7
  %tobool.not7 = icmp eq i8 %1, 0
  br i1 %tobool.not7, label %cleanup, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %2 = phi i8 [ %3, %if.end ], [ %1, %entry ]
  %s.addr.08 = phi ptr [ %incdec.ptr, %if.end ], [ %s, %entry ]
  %cmp = icmp eq i8 %2, %0
  br i1 %cmp, label %cleanup, label %if.end

if.end:                                           ; preds = %while.body
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.08, i16 1
  %3 = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %3, 0
  br i1 %tobool.not, label %cleanup, label %while.body, !llvm.loop !15

cleanup:                                          ; preds = %while.body, %if.end, %entry
  %retval.0 = phi ptr [ null, %entry ], [ null, %if.end ], [ %s.addr.08, %while.body ]
  ret ptr %retval.0
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local range(i16 -255, 256) i16 @test_strncmp(ptr noundef readonly captures(none) %s1, ptr noundef readonly captures(none) %s2, i16 noundef %n) local_unnamed_addr #3 {
entry:
  %tobool.not18 = icmp eq i16 %n, 0
  br i1 %tobool.not18, label %return, label %while.body

while.body:                                       ; preds = %entry, %if.end
  %dec21.in = phi i16 [ %dec21, %if.end ], [ %n, %entry ]
  %s2.addr.020 = phi ptr [ %incdec.ptr10, %if.end ], [ %s2, %entry ]
  %s1.addr.019 = phi ptr [ %incdec.ptr, %if.end ], [ %s1, %entry ]
  %0 = load i8, ptr %s1.addr.019, align 1, !tbaa !7
  %1 = load i8, ptr %s2.addr.020, align 1, !tbaa !7
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
  br i1 %or.cond, label %return, label %while.body, !llvm.loop !16

return:                                           ; preds = %if.end, %entry, %if.then
  %retval.0 = phi i16 [ %sub, %if.then ], [ 0, %entry ], [ 0, %if.end ]
  ret i16 %retval.0
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef i16 @test_strlen(ptr noundef %s) local_unnamed_addr #3 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.cond, %entry
  %p.0 = phi ptr [ %s, %entry ], [ %incdec.ptr, %while.cond ]
  %0 = load i8, ptr %p.0, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %0, 0
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %p.0, i16 1
  br i1 %tobool.not, label %while.end, label %while.cond, !llvm.loop !17

while.end:                                        ; preds = %while.cond
  %sub.ptr.lhs.cast = ptrtoint ptr %p.0 to i16
  %sub.ptr.rhs.cast = ptrtoint ptr %s to i16
  %sub.ptr.sub = sub i16 %sub.ptr.lhs.cast, %sub.ptr.rhs.cast
  ret i16 %sub.ptr.sub
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_strncpy(ptr noundef returned writeonly captures(ret: address, provenance) %dst, ptr noundef readonly captures(none) %src, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not15 = icmp eq i16 %n, 0
  br i1 %tobool.not15, label %while.end8, label %land.rhs

land.rhs:                                         ; preds = %entry, %while.body
  %d.018 = phi ptr [ %incdec.ptr, %while.body ], [ %dst, %entry ]
  %n.addr.017 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %src.addr.016 = phi ptr [ %incdec.ptr2, %while.body ], [ %src, %entry ]
  %0 = load i8, ptr %src.addr.016, align 1, !tbaa !7
  store i8 %0, ptr %d.018, align 1, !tbaa !7
  %tobool1.not = icmp eq i8 %0, 0
  br i1 %tobool1.not, label %while.body6, label %while.body

while.body:                                       ; preds = %land.rhs
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %d.018, i16 1
  %incdec.ptr2 = getelementptr inbounds nuw i8, ptr %src.addr.016, i16 1
  %dec = add i16 %n.addr.017, -1
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end8, label %land.rhs, !llvm.loop !18

while.body6:                                      ; preds = %land.rhs, %while.body6
  %d.124 = phi ptr [ %incdec.ptr7, %while.body6 ], [ %d.018, %land.rhs ]
  %n.addr.123 = phi i16 [ %dec4, %while.body6 ], [ %n.addr.017, %land.rhs ]
  %dec4 = add i16 %n.addr.123, -1
  %incdec.ptr7 = getelementptr inbounds nuw i8, ptr %d.124, i16 1
  store i8 0, ptr %d.124, align 1, !tbaa !7
  %tobool5.not = icmp eq i16 %dec4, 0
  br i1 %tobool5.not, label %while.end8, label %while.body6, !llvm.loop !19

while.end8:                                       ; preds = %while.body, %while.body6, %entry
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_strcat(ptr noundef returned captures(ret: address, provenance) %dst, ptr noundef readonly captures(none) %src) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.cond, %entry
  %d.0 = phi ptr [ %dst, %entry ], [ %incdec.ptr, %while.cond ]
  %0 = load i8, ptr %d.0, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %0, 0
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %d.0, i16 1
  br i1 %tobool.not, label %while.cond1, label %while.cond, !llvm.loop !20

while.cond1:                                      ; preds = %while.cond, %while.cond1
  %src.addr.0 = phi ptr [ %incdec.ptr2, %while.cond1 ], [ %src, %while.cond ]
  %d.1 = phi ptr [ %incdec.ptr3, %while.cond1 ], [ %d.0, %while.cond ]
  %incdec.ptr2 = getelementptr inbounds nuw i8, ptr %src.addr.0, i16 1
  %1 = load i8, ptr %src.addr.0, align 1, !tbaa !7
  %incdec.ptr3 = getelementptr inbounds nuw i8, ptr %d.1, i16 1
  store i8 %1, ptr %d.1, align 1, !tbaa !7
  %tobool4.not = icmp eq i8 %1, 0
  br i1 %tobool4.not, label %while.end6, label %while.cond1, !llvm.loop !21

while.end6:                                       ; preds = %while.cond1
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local noundef ptr @test_strncat(ptr noundef returned captures(ret: address, provenance) %dst, ptr noundef readonly captures(none) %src, i16 noundef %n) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.cond, %entry
  %d.0 = phi ptr [ %dst, %entry ], [ %incdec.ptr, %while.cond ]
  %0 = load i8, ptr %d.0, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %0, 0
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %d.0, i16 1
  br i1 %tobool.not, label %while.cond1.preheader, label %while.cond, !llvm.loop !22

while.cond1.preheader:                            ; preds = %while.cond
  %tobool2.not14 = icmp eq i16 %n, 0
  br i1 %tobool2.not14, label %while.end7, label %land.rhs

land.rhs:                                         ; preds = %while.cond1.preheader, %while.body4
  %d.117 = phi ptr [ %incdec.ptr6, %while.body4 ], [ %d.0, %while.cond1.preheader ]
  %n.addr.016 = phi i16 [ %dec, %while.body4 ], [ %n, %while.cond1.preheader ]
  %src.addr.015 = phi ptr [ %incdec.ptr5, %while.body4 ], [ %src, %while.cond1.preheader ]
  %1 = load i8, ptr %src.addr.015, align 1, !tbaa !7
  %tobool3.not = icmp eq i8 %1, 0
  br i1 %tobool3.not, label %while.end7, label %while.body4

while.body4:                                      ; preds = %land.rhs
  %incdec.ptr5 = getelementptr inbounds nuw i8, ptr %src.addr.015, i16 1
  %incdec.ptr6 = getelementptr inbounds nuw i8, ptr %d.117, i16 1
  store i8 %1, ptr %d.117, align 1, !tbaa !7
  %dec = add i16 %n.addr.016, -1
  %tobool2.not = icmp eq i16 %dec, 0
  br i1 %tobool2.not, label %while.end7, label %land.rhs, !llvm.loop !23

while.end7:                                       ; preds = %land.rhs, %while.body4, %while.cond1.preheader
  %d.1.lcssa = phi ptr [ %d.0, %while.cond1.preheader ], [ %incdec.ptr6, %while.body4 ], [ %d.117, %land.rhs ]
  store i8 0, ptr %d.1.lcssa, align 1, !tbaa !7
  ret ptr %dst
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local ptr @test_strrchr(ptr noundef readonly captures(ret: address, provenance) %s, i16 noundef %c) local_unnamed_addr #3 {
entry:
  %0 = trunc i16 %c to i8
  br label %do.body

do.body:                                          ; preds = %do.body, %entry
  %last.0 = phi ptr [ null, %entry ], [ %last.1, %do.body ]
  %s.addr.0 = phi ptr [ %s, %entry ], [ %incdec.ptr, %do.body ]
  %1 = load i8, ptr %s.addr.0, align 1, !tbaa !7
  %cmp = icmp eq i8 %1, %0
  %last.1 = select i1 %cmp, ptr %s.addr.0, ptr %last.0
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.0, i16 1
  %tobool.not = icmp eq i8 %1, 0
  br i1 %tobool.not, label %do.end, label %do.body, !llvm.loop !24

do.end:                                           ; preds = %do.body
  ret ptr %last.1
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef ptr @test_strstr(ptr noundef readonly captures(ret: address, provenance) %haystack, ptr noundef readonly captures(none) %needle) local_unnamed_addr #3 {
entry:
  %0 = load i8, ptr %needle, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %0, 0
  br i1 %tobool.not, label %return, label %for.cond.preheader

for.cond.preheader:                               ; preds = %entry
  %1 = load i8, ptr %haystack, align 1, !tbaa !7
  %tobool1.not26 = icmp eq i8 %1, 0
  br i1 %tobool1.not26, label %return, label %while.cond.preheader

while.cond.preheader:                             ; preds = %for.cond.preheader, %for.inc
  %2 = phi i8 [ %6, %for.inc ], [ %1, %for.cond.preheader ]
  %haystack.addr.027 = phi ptr [ %incdec.ptr13, %for.inc ], [ %haystack, %for.cond.preheader ]
  br label %land.lhs.true

land.lhs.true:                                    ; preds = %while.cond.preheader, %while.body
  %n.025 = phi ptr [ %needle, %while.cond.preheader ], [ %incdec.ptr8, %while.body ]
  %h.024 = phi ptr [ %haystack.addr.027, %while.cond.preheader ], [ %incdec.ptr, %while.body ]
  %3 = phi i8 [ %2, %while.cond.preheader ], [ %.pr, %while.body ]
  %4 = load i8, ptr %n.025, align 1, !tbaa !7
  %cmp = icmp eq i8 %3, %4
  br i1 %cmp, label %while.body, label %while.end

while.body:                                       ; preds = %land.lhs.true
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %h.024, i16 1
  %incdec.ptr8 = getelementptr inbounds nuw i8, ptr %n.025, i16 1
  %.pr = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %tobool2.not = icmp eq i8 %.pr, 0
  br i1 %tobool2.not, label %while.body.while.end_crit_edge, label %land.lhs.true, !llvm.loop !25

while.body.while.end_crit_edge:                   ; preds = %while.body
  %.pre = load i8, ptr %incdec.ptr8, align 1, !tbaa !7
  br label %while.end, !llvm.loop !25

while.end:                                        ; preds = %land.lhs.true, %while.body.while.end_crit_edge
  %5 = phi i8 [ %.pre, %while.body.while.end_crit_edge ], [ %4, %land.lhs.true ]
  %tobool9.not.not = icmp eq i8 %5, 0
  br i1 %tobool9.not.not, label %return, label %for.inc

for.inc:                                          ; preds = %while.end
  %incdec.ptr13 = getelementptr inbounds nuw i8, ptr %haystack.addr.027, i16 1
  %6 = load i8, ptr %incdec.ptr13, align 1, !tbaa !7
  %tobool1.not = icmp eq i8 %6, 0
  br i1 %tobool1.not, label %return, label %while.cond.preheader, !llvm.loop !26

return:                                           ; preds = %for.inc, %while.end, %for.cond.preheader, %entry
  %retval.2 = phi ptr [ %haystack, %entry ], [ null, %for.cond.preheader ], [ null, %for.inc ], [ %haystack.addr.027, %while.end ]
  ret ptr %retval.2
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef i16 @test_strspn(ptr noundef %s, ptr noundef readonly captures(none) %accept) local_unnamed_addr #3 {
entry:
  %0 = load i8, ptr %s, align 1, !tbaa !7
  %tobool.not26 = icmp eq i8 %0, 0
  br i1 %tobool.not26, label %while.end9, label %while.cond1.preheader.lr.ph

while.cond1.preheader.lr.ph:                      ; preds = %entry
  %1 = load i8, ptr %accept, align 1, !tbaa !7
  %tobool2.not23 = icmp eq i8 %1, 0
  br i1 %tobool2.not23, label %while.end9, label %while.cond1.preheader

while.cond1.preheader:                            ; preds = %while.cond1.preheader.lr.ph, %while.end
  %2 = phi i8 [ %5, %while.end ], [ %0, %while.cond1.preheader.lr.ph ]
  %p.027 = phi ptr [ %incdec.ptr8, %while.end ], [ %s, %while.cond1.preheader.lr.ph ]
  br label %land.rhs

while.cond1:                                      ; preds = %land.rhs
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %a.024, i16 1
  %3 = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %tobool2.not = icmp eq i8 %3, 0
  br i1 %tobool2.not, label %while.end9, label %land.rhs, !llvm.loop !27

land.rhs:                                         ; preds = %while.cond1.preheader, %while.cond1
  %4 = phi i8 [ %1, %while.cond1.preheader ], [ %3, %while.cond1 ]
  %a.024 = phi ptr [ %accept, %while.cond1.preheader ], [ %incdec.ptr, %while.cond1 ]
  %cmp.not = icmp eq i8 %4, %2
  br i1 %cmp.not, label %while.end, label %while.cond1

while.end:                                        ; preds = %land.rhs
  %incdec.ptr8 = getelementptr inbounds nuw i8, ptr %p.027, i16 1
  %5 = load i8, ptr %incdec.ptr8, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %5, 0
  br i1 %tobool.not, label %while.end9, label %while.cond1.preheader

while.end9:                                       ; preds = %while.end, %while.cond1, %entry, %while.cond1.preheader.lr.ph
  %p.022 = phi ptr [ %p.027, %while.cond1 ], [ %s, %while.cond1.preheader.lr.ph ], [ %s, %entry ], [ %incdec.ptr8, %while.end ]
  %sub.ptr.lhs.cast = ptrtoint ptr %p.022 to i16
  %sub.ptr.rhs.cast = ptrtoint ptr %s to i16
  %sub.ptr.sub = sub i16 %sub.ptr.lhs.cast, %sub.ptr.rhs.cast
  ret i16 %sub.ptr.sub
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef i16 @test_strcspn(ptr noundef %s, ptr noundef readonly captures(none) %reject) local_unnamed_addr #3 {
entry:
  %0 = load i8, ptr %s, align 1, !tbaa !7
  %tobool.not25 = icmp eq i8 %0, 0
  br i1 %tobool.not25, label %while.end9, label %while.cond1.preheader.lr.ph

while.cond1.preheader.lr.ph:                      ; preds = %entry
  %1 = load i8, ptr %reject, align 1, !tbaa !7
  %tobool2.not23 = icmp eq i8 %1, 0
  br i1 %tobool2.not23, label %while.cond1.preheader.us, label %while.cond1.preheader

while.cond1.preheader.us:                         ; preds = %while.cond1.preheader.lr.ph, %while.cond1.preheader.us
  %p.026.us = phi ptr [ %incdec.ptr8.us, %while.cond1.preheader.us ], [ %s, %while.cond1.preheader.lr.ph ]
  %incdec.ptr8.us = getelementptr inbounds nuw i8, ptr %p.026.us, i16 1
  %2 = load i8, ptr %incdec.ptr8.us, align 1, !tbaa !7
  %tobool.not.us = icmp eq i8 %2, 0
  br i1 %tobool.not.us, label %while.end9, label %while.cond1.preheader.us

while.cond1.preheader:                            ; preds = %while.cond1.preheader.lr.ph, %while.cond1.while.end_crit_edge
  %3 = phi i8 [ %6, %while.cond1.while.end_crit_edge ], [ %0, %while.cond1.preheader.lr.ph ]
  %p.026 = phi ptr [ %incdec.ptr8, %while.cond1.while.end_crit_edge ], [ %s, %while.cond1.preheader.lr.ph ]
  br label %land.rhs

while.cond1:                                      ; preds = %land.rhs
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %r.024, i16 1
  %4 = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %tobool2.not = icmp eq i8 %4, 0
  br i1 %tobool2.not, label %while.cond1.while.end_crit_edge, label %land.rhs, !llvm.loop !28

land.rhs:                                         ; preds = %while.cond1.preheader, %while.cond1
  %5 = phi i8 [ %1, %while.cond1.preheader ], [ %4, %while.cond1 ]
  %r.024 = phi ptr [ %reject, %while.cond1.preheader ], [ %incdec.ptr, %while.cond1 ]
  %cmp.not = icmp eq i8 %5, %3
  br i1 %cmp.not, label %while.end9, label %while.cond1

while.cond1.while.end_crit_edge:                  ; preds = %while.cond1
  %incdec.ptr8 = getelementptr inbounds nuw i8, ptr %p.026, i16 1
  %6 = load i8, ptr %incdec.ptr8, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %6, 0
  br i1 %tobool.not, label %while.end9, label %while.cond1.preheader

while.end9:                                       ; preds = %while.cond1.while.end_crit_edge, %land.rhs, %while.cond1.preheader.us, %entry
  %p.022 = phi ptr [ %p.026, %land.rhs ], [ %s, %entry ], [ %incdec.ptr8.us, %while.cond1.preheader.us ], [ %incdec.ptr8, %while.cond1.while.end_crit_edge ]
  %sub.ptr.lhs.cast = ptrtoint ptr %p.022 to i16
  %sub.ptr.rhs.cast = ptrtoint ptr %s to i16
  %sub.ptr.sub = sub i16 %sub.ptr.lhs.cast, %sub.ptr.rhs.cast
  ret i16 %sub.ptr.sub
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef ptr @test_strpbrk(ptr noundef readonly captures(ret: address, provenance) %s, ptr noundef readonly captures(none) %accept) local_unnamed_addr #3 {
entry:
  %0 = load i8, ptr %s, align 1, !tbaa !7
  %tobool.not19 = icmp eq i8 %0, 0
  br i1 %tobool.not19, label %return, label %while.cond1.preheader.lr.ph

while.cond1.preheader.lr.ph:                      ; preds = %entry
  %1 = load i8, ptr %accept, align 1, !tbaa !7
  %tobool2.not.not17 = icmp eq i8 %1, 0
  br i1 %tobool2.not.not17, label %while.cond1.preheader.us, label %while.cond1.preheader

while.cond1.preheader.us:                         ; preds = %while.cond1.preheader.lr.ph, %while.cond1.preheader.us
  %s.addr.020.us = phi ptr [ %incdec.ptr6.us, %while.cond1.preheader.us ], [ %s, %while.cond1.preheader.lr.ph ]
  %incdec.ptr6.us = getelementptr inbounds nuw i8, ptr %s.addr.020.us, i16 1
  %2 = load i8, ptr %incdec.ptr6.us, align 1, !tbaa !7
  %tobool.not.us = icmp eq i8 %2, 0
  br i1 %tobool.not.us, label %return, label %while.cond1.preheader.us

while.cond1.preheader:                            ; preds = %while.cond1.preheader.lr.ph, %while.cond1.cleanup_crit_edge
  %3 = phi i8 [ %6, %while.cond1.cleanup_crit_edge ], [ %0, %while.cond1.preheader.lr.ph ]
  %s.addr.020 = phi ptr [ %incdec.ptr6, %while.cond1.cleanup_crit_edge ], [ %s, %while.cond1.preheader.lr.ph ]
  br label %while.body3

while.cond1:                                      ; preds = %while.body3
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %a.018, i16 1
  %4 = load i8, ptr %incdec.ptr, align 1, !tbaa !7
  %tobool2.not.not = icmp eq i8 %4, 0
  br i1 %tobool2.not.not, label %while.cond1.cleanup_crit_edge, label %while.body3, !llvm.loop !29

while.body3:                                      ; preds = %while.cond1.preheader, %while.cond1
  %5 = phi i8 [ %1, %while.cond1.preheader ], [ %4, %while.cond1 ]
  %a.018 = phi ptr [ %accept, %while.cond1.preheader ], [ %incdec.ptr, %while.cond1 ]
  %cmp = icmp eq i8 %5, %3
  br i1 %cmp, label %return, label %while.cond1

while.cond1.cleanup_crit_edge:                    ; preds = %while.cond1
  %incdec.ptr6 = getelementptr inbounds nuw i8, ptr %s.addr.020, i16 1
  %6 = load i8, ptr %incdec.ptr6, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %6, 0
  br i1 %tobool.not, label %return, label %while.cond1.preheader

return:                                           ; preds = %while.cond1.cleanup_crit_edge, %while.body3, %while.cond1.preheader.us, %entry
  %retval.2 = phi ptr [ %s.addr.020, %while.body3 ], [ null, %entry ], [ null, %while.cond1.preheader.us ], [ null, %while.cond1.cleanup_crit_edge ]
  ret ptr %retval.2
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local noundef ptr @test_memchr(ptr noundef readonly captures(ret: address, provenance) %s, i16 noundef %c, i16 noundef %n) local_unnamed_addr #3 {
entry:
  %tobool.not8 = icmp eq i16 %n, 0
  br i1 %tobool.not8, label %cleanup, label %while.body.lr.ph

while.body.lr.ph:                                 ; preds = %entry
  %0 = trunc i16 %c to i8
  br label %while.body

while.body:                                       ; preds = %while.body.lr.ph, %if.end
  %dec10.in = phi i16 [ %n, %while.body.lr.ph ], [ %dec10, %if.end ]
  %p.09 = phi ptr [ %s, %while.body.lr.ph ], [ %incdec.ptr, %if.end ]
  %1 = load i8, ptr %p.09, align 1, !tbaa !7
  %cmp = icmp eq i8 %1, %0
  br i1 %cmp, label %cleanup, label %if.end

if.end:                                           ; preds = %while.body
  %dec10 = add i16 %dec10.in, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %p.09, i16 1
  %tobool.not = icmp eq i16 %dec10, 0
  br i1 %tobool.not, label %cleanup, label %while.body, !llvm.loop !30

cleanup:                                          ; preds = %while.body, %if.end, %entry
  %retval.0 = phi ptr [ null, %entry ], [ null, %if.end ], [ %p.09, %while.body ]
  ret ptr %retval.0
}

; Function Attrs: nofree norecurse nosync nounwind memory(readwrite, inaccessiblemem: none, target_mem0: none, target_mem1: none)
define dso_local noundef ptr @test_strtok(ptr noundef %str, ptr noundef readonly captures(none) %delim) local_unnamed_addr #4 {
entry:
  %tobool.not = icmp eq ptr %str, null
  br i1 %tobool.not, label %if.end, label %for.cond.preheader

if.end:                                           ; preds = %entry
  %.pr = load ptr, ptr @test_strtok.next, align 1, !tbaa !31
  %tobool1.not = icmp eq ptr %.pr, null
  br i1 %tobool1.not, label %return, label %for.cond.preheader

for.cond.preheader:                               ; preds = %entry, %if.end
  %start.0.ph = phi ptr [ %str, %entry ], [ %.pr, %if.end ]
  br label %for.cond

for.cond:                                         ; preds = %for.cond.preheader, %while.end
  %start.0 = phi ptr [ %incdec.ptr14, %while.end ], [ %start.0.ph, %for.cond.preheader ]
  %0 = load i8, ptr %start.0, align 1, !tbaa !7
  %tobool4.not = icmp eq i8 %0, 0
  br i1 %tobool4.not, label %return.sink.split, label %while.cond

while.cond:                                       ; preds = %for.cond, %while.cond
  %d.0 = phi ptr [ %incdec.ptr, %while.cond ], [ %delim, %for.cond ]
  %1 = load i8, ptr %d.0, align 1, !tbaa !7
  %tobool7.not = icmp eq i8 %1, 0
  %cmp.not = icmp eq i8 %1, %0
  %or.cond = or i1 %tobool7.not, %cmp.not
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %d.0, i16 1
  br i1 %or.cond, label %while.end, label %while.cond, !llvm.loop !34

while.end:                                        ; preds = %while.cond
  %incdec.ptr14 = getelementptr inbounds nuw i8, ptr %start.0, i16 1
  br i1 %tobool7.not, label %while.cond20.preheader.lr.ph, label %for.cond

while.cond20.preheader.lr.ph:                     ; preds = %while.end
  %2 = load i8, ptr %delim, align 1, !tbaa !7
  %tobool22.not63 = icmp eq i8 %2, 0
  br i1 %tobool22.not63, label %while.cond20.preheader.us, label %while.cond20.preheader

while.cond20.preheader.us:                        ; preds = %while.cond20.preheader.lr.ph, %while.cond20.preheader.us
  %end.066.us = phi ptr [ %incdec.ptr35.us, %while.cond20.preheader.us ], [ %start.0, %while.cond20.preheader.lr.ph ]
  %incdec.ptr35.us = getelementptr inbounds nuw i8, ptr %end.066.us, i16 1
  %3 = load i8, ptr %incdec.ptr35.us, align 1, !tbaa !7
  %tobool16.not.us = icmp eq i8 %3, 0
  br i1 %tobool16.not.us, label %return.sink.split, label %while.cond20.preheader.us

while.cond20.preheader:                           ; preds = %while.cond20.preheader.lr.ph, %while.cond20.cleanup36_crit_edge
  %4 = phi i8 [ %7, %while.cond20.cleanup36_crit_edge ], [ %0, %while.cond20.preheader.lr.ph ]
  %end.066 = phi ptr [ %incdec.ptr35, %while.cond20.cleanup36_crit_edge ], [ %start.0, %while.cond20.preheader.lr.ph ]
  br label %land.rhs23

while.cond20:                                     ; preds = %land.rhs23
  %incdec.ptr30 = getelementptr inbounds nuw i8, ptr %d19.064, i16 1
  %5 = load i8, ptr %incdec.ptr30, align 1, !tbaa !7
  %tobool22.not = icmp eq i8 %5, 0
  br i1 %tobool22.not, label %while.cond20.cleanup36_crit_edge, label %land.rhs23, !llvm.loop !35

land.rhs23:                                       ; preds = %while.cond20.preheader, %while.cond20
  %6 = phi i8 [ %2, %while.cond20.preheader ], [ %5, %while.cond20 ]
  %d19.064 = phi ptr [ %delim, %while.cond20.preheader ], [ %incdec.ptr30, %while.cond20 ]
  %cmp26.not = icmp eq i8 %6, %4
  br i1 %cmp26.not, label %cleanup36.thread, label %while.cond20

cleanup36.thread:                                 ; preds = %land.rhs23
  store i8 0, ptr %end.066, align 1, !tbaa !7
  %add.ptr = getelementptr inbounds nuw i8, ptr %end.066, i16 1
  br label %return.sink.split

while.cond20.cleanup36_crit_edge:                 ; preds = %while.cond20
  %incdec.ptr35 = getelementptr inbounds nuw i8, ptr %end.066, i16 1
  %7 = load i8, ptr %incdec.ptr35, align 1, !tbaa !7
  %tobool16.not = icmp eq i8 %7, 0
  br i1 %tobool16.not, label %return.sink.split, label %while.cond20.preheader

return.sink.split:                                ; preds = %for.cond, %while.cond20.cleanup36_crit_edge, %while.cond20.preheader.us, %cleanup36.thread
  %add.ptr.sink = phi ptr [ %add.ptr, %cleanup36.thread ], [ null, %while.cond20.preheader.us ], [ null, %while.cond20.cleanup36_crit_edge ], [ null, %for.cond ]
  %retval.4.ph = phi ptr [ %start.0, %cleanup36.thread ], [ %start.0, %while.cond20.preheader.us ], [ %start.0, %while.cond20.cleanup36_crit_edge ], [ null, %for.cond ]
  store ptr %add.ptr.sink, ptr @test_strtok.next, align 1, !tbaa !31
  br label %return

return:                                           ; preds = %return.sink.split, %if.end
  %retval.4 = phi ptr [ null, %if.end ], [ %retval.4.ph, %return.sink.split ]
  ret ptr %retval.4
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isspace(i16 noundef %c) local_unnamed_addr #5 {
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
define dso_local range(i16 0, 2) i16 @test_isprint(i16 noundef %c) local_unnamed_addr #5 {
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
define dso_local range(i16 0, 2) i16 @test_isxdigit(i16 noundef %c) local_unnamed_addr #5 {
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
define dso_local i16 @test_tolower(i16 noundef %c) local_unnamed_addr #5 {
entry:
  %0 = add i16 %c, -65
  %or.cond = icmp ult i16 %0, 26
  %add = or disjoint i16 %c, 32
  %cond = select i1 %or.cond, i16 %add, i16 %c
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local i16 @test_toupper(i16 noundef %c) local_unnamed_addr #5 {
entry:
  %0 = add i16 %c, -97
  %or.cond = icmp ult i16 %0, 26
  %sub = add nsw i16 %c, -32
  %cond = select i1 %or.cond, i16 %sub, i16 %c
  ret i16 %cond
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_ispunct(i16 noundef %c) local_unnamed_addr #5 {
entry:
  %0 = add i16 %c, -33
  %or.cond = icmp ult i16 %0, 15
  br i1 %or.cond, label %lor.end, label %switch.early.test

switch.early.test:                                ; preds = %entry
  switch i16 %c, label %lor.rhs [
    i16 96, label %lor.end
    i16 95, label %lor.end
    i16 94, label %lor.end
    i16 93, label %lor.end
    i16 92, label %lor.end
    i16 91, label %lor.end
    i16 64, label %lor.end
    i16 63, label %lor.end
    i16 62, label %lor.end
    i16 61, label %lor.end
    i16 60, label %lor.end
    i16 59, label %lor.end
    i16 58, label %lor.end
  ]

lor.rhs:                                          ; preds = %switch.early.test
  %cmp9 = icmp sgt i16 %c, 122
  br i1 %cmp9, label %land.rhs, label %lor.end

land.rhs:                                         ; preds = %lor.rhs
  %cmp10 = icmp samesign ult i16 %c, 127
  %1 = zext i1 %cmp10 to i16
  br label %lor.end

lor.end:                                          ; preds = %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %switch.early.test, %entry, %lor.rhs, %land.rhs
  %lor.ext = phi i16 [ 0, %lor.rhs ], [ %1, %land.rhs ], [ 1, %switch.early.test ], [ 1, %entry ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ], [ 1, %switch.early.test ]
  ret i16 %lor.ext
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_iscntrl(i16 noundef %c) local_unnamed_addr #5 {
entry:
  %or.cond = icmp ult i16 %c, 32
  br i1 %or.cond, label %lor.end, label %lor.rhs

lor.rhs:                                          ; preds = %entry
  %cmp2 = icmp eq i16 %c, 127
  %0 = zext i1 %cmp2 to i16
  br label %lor.end

lor.end:                                          ; preds = %entry, %lor.rhs
  %lor.ext = phi i16 [ 1, %entry ], [ %0, %lor.rhs ]
  ret i16 %lor.ext
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(none)
define dso_local range(i16 0, 2) i16 @test_isgraph(i16 noundef %c) local_unnamed_addr #5 {
entry:
  %cmp = icmp sgt i16 %c, 32
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
define dso_local range(i32 0, -2147483648) i32 @test_labs(i32 noundef %x) local_unnamed_addr #5 {
entry:
  %cond = tail call i32 @llvm.abs.i32(i32 %x, i1 true)
  ret i32 %cond
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local i32 @test_atol(ptr noundef readonly captures(none) %s) local_unnamed_addr #3 {
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
  br i1 %or.cond, label %while.body, label %while.end, !llvm.loop !36

while.end:                                        ; preds = %while.body, %if.end7
  %result.0.lcssa = phi i32 [ 0, %if.end7 ], [ %add, %while.body ]
  %sub19 = sub nsw i32 0, %result.0.lcssa
  %cond = select i1 %cmp, i32 %sub19, i32 %result.0.lcssa
  ret i32 %cond
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local range(i32 -2147483647, -2147483648) i32 @test_strtol(ptr noundef %nptr, ptr noundef writeonly captures(address_is_null) %endptr, i16 noundef %base) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.body, %entry
  %s.0 = phi ptr [ %nptr, %entry ], [ %incdec.ptr, %while.body ]
  %0 = load i8, ptr %s.0, align 1, !tbaa !7
  switch i8 %0, label %while.end [
    i8 32, label %while.body
    i8 9, label %while.body
    i8 10, label %while.body
  ]

while.body:                                       ; preds = %while.cond, %while.cond, %while.cond
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %while.cond, !llvm.loop !37

while.end:                                        ; preds = %while.cond
  %cmp9.not = icmp eq i8 %0, 45
  br i1 %cmp9.not, label %if.then, label %if.else

if.then:                                          ; preds = %while.end
  %incdec.ptr11 = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %if.end17

if.else:                                          ; preds = %while.end
  %cmp13 = icmp eq i8 %0, 43
  br i1 %cmp13, label %if.then15, label %if.end17

if.then15:                                        ; preds = %if.else
  %incdec.ptr16 = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %if.end17

if.end17:                                         ; preds = %if.else, %if.then15, %if.then
  %s.1 = phi ptr [ %incdec.ptr11, %if.then ], [ %incdec.ptr16, %if.then15 ], [ %s.0, %if.else ]
  %cmp18 = icmp eq i16 %base, 0
  switch i16 %base, label %if.end45 [
    i16 16, label %land.lhs.true
    i16 0, label %land.lhs.true
  ]

land.lhs.true:                                    ; preds = %if.end17, %if.end17
  %1 = load i8, ptr %s.1, align 1, !tbaa !7
  %cmp24 = icmp eq i8 %1, 48
  br i1 %cmp24, label %land.lhs.true26, label %if.else37

land.lhs.true26:                                  ; preds = %land.lhs.true
  %arrayidx27 = getelementptr inbounds nuw i8, ptr %s.1, i16 1
  %2 = load i8, ptr %arrayidx27, align 1, !tbaa !7
  switch i8 %2, label %if.else37 [
    i8 120, label %if.then36
    i8 88, label %if.then36
  ]

if.then36:                                        ; preds = %land.lhs.true26, %land.lhs.true26
  %add.ptr = getelementptr inbounds nuw i8, ptr %s.1, i16 2
  br label %if.end45

if.else37:                                        ; preds = %land.lhs.true26, %land.lhs.true
  br i1 %cmp18, label %if.then40, label %if.end45

if.then40:                                        ; preds = %if.else37
  %cond = select i1 %cmp24, i16 8, i16 10
  br label %if.end45

if.end45:                                         ; preds = %if.end17, %if.else37, %if.then40, %if.then36
  %base.addr.0 = phi i16 [ 16, %if.then36 ], [ %cond, %if.then40 ], [ %base, %if.else37 ], [ %base, %if.end17 ]
  %s.2 = phi ptr [ %add.ptr, %if.then36 ], [ %s.1, %if.then40 ], [ %s.1, %if.else37 ], [ %s.1, %if.end17 ]
  %3 = load i8, ptr %s.2, align 1, !tbaa !7
  %tobool.not126 = icmp eq i8 %3, 0
  br i1 %tobool.not126, label %while.end82, label %while.body47.lr.ph

while.body47.lr.ph:                               ; preds = %if.end45
  %conv79 = zext nneg i16 %base.addr.0 to i32
  br label %while.body47

while.body47:                                     ; preds = %while.body47.lr.ph, %if.end78
  %4 = phi i8 [ %3, %while.body47.lr.ph ], [ %8, %if.end78 ]
  %acc.0128 = phi i32 [ 0, %while.body47.lr.ph ], [ %add, %if.end78 ]
  %s.3127 = phi ptr [ %s.2, %while.body47.lr.ph ], [ %incdec.ptr81, %if.end78 ]
  %conv48 = zext i8 %4 to i16
  %5 = add i8 %4, -48
  %or.cond90 = icmp ult i8 %5, 10
  br i1 %or.cond90, label %if.end74, label %if.else55

if.else55:                                        ; preds = %while.body47
  %6 = add i8 %4, -97
  %or.cond91 = icmp ult i8 %6, 26
  br i1 %or.cond91, label %if.end74, label %if.else63

if.else63:                                        ; preds = %if.else55
  %7 = add i8 %4, -65
  %or.cond92 = icmp ult i8 %7, 26
  br i1 %or.cond92, label %if.end74, label %while.end82

if.end74:                                         ; preds = %if.else63, %if.else55, %while.body47
  %.sink = phi i16 [ -48, %while.body47 ], [ -87, %if.else55 ], [ -55, %if.else63 ]
  %sub62 = add nsw i16 %.sink, %conv48
  %cmp75.not = icmp slt i16 %sub62, %base.addr.0
  br i1 %cmp75.not, label %if.end78, label %while.end82

if.end78:                                         ; preds = %if.end74
  %mul = mul nsw i32 %acc.0128, %conv79
  %conv80 = zext nneg i16 %sub62 to i32
  %add = add nuw nsw i32 %mul, %conv80
  %incdec.ptr81 = getelementptr inbounds nuw i8, ptr %s.3127, i16 1
  %8 = load i8, ptr %incdec.ptr81, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %8, 0
  br i1 %tobool.not, label %while.end82, label %while.body47, !llvm.loop !38

while.end82:                                      ; preds = %if.end78, %if.else63, %if.end74, %if.end45
  %s.3.lcssa = phi ptr [ %s.2, %if.end45 ], [ %s.3127, %if.end74 ], [ %s.3127, %if.else63 ], [ %incdec.ptr81, %if.end78 ]
  %acc.0.lcssa = phi i32 [ 0, %if.end45 ], [ %acc.0128, %if.end74 ], [ %acc.0128, %if.else63 ], [ %add, %if.end78 ]
  %cmp83.not = icmp eq ptr %endptr, null
  br i1 %cmp83.not, label %if.end86, label %if.then85

if.then85:                                        ; preds = %while.end82
  store ptr %s.3.lcssa, ptr %endptr, align 1, !tbaa !31
  br label %if.end86

if.end86:                                         ; preds = %if.then85, %while.end82
  %sub88 = sub nsw i32 0, %acc.0.lcssa
  %cond89 = select i1 %cmp9.not, i32 %sub88, i32 %acc.0.lcssa
  ret i32 %cond89
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local i32 @test_strtoul(ptr noundef %nptr, ptr noundef writeonly captures(address_is_null) %endptr, i16 noundef %base) local_unnamed_addr #0 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.body, %entry
  %s.0 = phi ptr [ %nptr, %entry ], [ %incdec.ptr, %while.body ]
  %0 = load i8, ptr %s.0, align 1, !tbaa !7
  switch i8 %0, label %while.end [
    i8 32, label %while.body
    i8 9, label %while.body
    i8 10, label %while.body
  ]

while.body:                                       ; preds = %while.cond, %while.cond, %while.cond
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %while.cond, !llvm.loop !39

while.end:                                        ; preds = %while.cond
  %cmp9.not = icmp eq i8 %0, 45
  br i1 %cmp9.not, label %if.then, label %if.else

if.then:                                          ; preds = %while.end
  %incdec.ptr11 = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %if.end17

if.else:                                          ; preds = %while.end
  %cmp13 = icmp eq i8 %0, 43
  br i1 %cmp13, label %if.then15, label %if.end17

if.then15:                                        ; preds = %if.else
  %incdec.ptr16 = getelementptr inbounds nuw i8, ptr %s.0, i16 1
  br label %if.end17

if.end17:                                         ; preds = %if.else, %if.then15, %if.then
  %s.1 = phi ptr [ %incdec.ptr11, %if.then ], [ %incdec.ptr16, %if.then15 ], [ %s.0, %if.else ]
  %cmp18 = icmp eq i16 %base, 0
  switch i16 %base, label %if.end45 [
    i16 16, label %land.lhs.true
    i16 0, label %land.lhs.true
  ]

land.lhs.true:                                    ; preds = %if.end17, %if.end17
  %1 = load i8, ptr %s.1, align 1, !tbaa !7
  %cmp24 = icmp eq i8 %1, 48
  br i1 %cmp24, label %land.lhs.true26, label %if.else37

land.lhs.true26:                                  ; preds = %land.lhs.true
  %arrayidx27 = getelementptr inbounds nuw i8, ptr %s.1, i16 1
  %2 = load i8, ptr %arrayidx27, align 1, !tbaa !7
  switch i8 %2, label %if.else37 [
    i8 120, label %if.then36
    i8 88, label %if.then36
  ]

if.then36:                                        ; preds = %land.lhs.true26, %land.lhs.true26
  %add.ptr = getelementptr inbounds nuw i8, ptr %s.1, i16 2
  br label %if.end45

if.else37:                                        ; preds = %land.lhs.true26, %land.lhs.true
  br i1 %cmp18, label %if.then40, label %if.end45

if.then40:                                        ; preds = %if.else37
  %cond = select i1 %cmp24, i16 8, i16 10
  br label %if.end45

if.end45:                                         ; preds = %if.end17, %if.else37, %if.then40, %if.then36
  %base.addr.0 = phi i16 [ 16, %if.then36 ], [ %cond, %if.then40 ], [ %base, %if.else37 ], [ %base, %if.end17 ]
  %s.2 = phi ptr [ %add.ptr, %if.then36 ], [ %s.1, %if.then40 ], [ %s.1, %if.else37 ], [ %s.1, %if.end17 ]
  %3 = load i8, ptr %s.2, align 1, !tbaa !7
  %tobool.not126 = icmp eq i8 %3, 0
  br i1 %tobool.not126, label %while.end82, label %while.body47.lr.ph

while.body47.lr.ph:                               ; preds = %if.end45
  %conv79 = zext nneg i16 %base.addr.0 to i32
  br label %while.body47

while.body47:                                     ; preds = %while.body47.lr.ph, %if.end78
  %4 = phi i8 [ %3, %while.body47.lr.ph ], [ %8, %if.end78 ]
  %acc.0128 = phi i32 [ 0, %while.body47.lr.ph ], [ %add, %if.end78 ]
  %s.3127 = phi ptr [ %s.2, %while.body47.lr.ph ], [ %incdec.ptr81, %if.end78 ]
  %conv48 = zext i8 %4 to i16
  %5 = add i8 %4, -48
  %or.cond90 = icmp ult i8 %5, 10
  br i1 %or.cond90, label %if.end74, label %if.else55

if.else55:                                        ; preds = %while.body47
  %6 = add i8 %4, -97
  %or.cond91 = icmp ult i8 %6, 26
  br i1 %or.cond91, label %if.end74, label %if.else63

if.else63:                                        ; preds = %if.else55
  %7 = add i8 %4, -65
  %or.cond92 = icmp ult i8 %7, 26
  br i1 %or.cond92, label %if.end74, label %while.end82

if.end74:                                         ; preds = %if.else63, %if.else55, %while.body47
  %.sink = phi i16 [ -48, %while.body47 ], [ -87, %if.else55 ], [ -55, %if.else63 ]
  %sub62 = add nsw i16 %.sink, %conv48
  %cmp75.not = icmp slt i16 %sub62, %base.addr.0
  br i1 %cmp75.not, label %if.end78, label %while.end82

if.end78:                                         ; preds = %if.end74
  %mul = mul i32 %acc.0128, %conv79
  %conv80 = zext nneg i16 %sub62 to i32
  %add = add i32 %mul, %conv80
  %incdec.ptr81 = getelementptr inbounds nuw i8, ptr %s.3127, i16 1
  %8 = load i8, ptr %incdec.ptr81, align 1, !tbaa !7
  %tobool.not = icmp eq i8 %8, 0
  br i1 %tobool.not, label %while.end82, label %while.body47, !llvm.loop !40

while.end82:                                      ; preds = %if.end78, %if.else63, %if.end74, %if.end45
  %s.3.lcssa = phi ptr [ %s.2, %if.end45 ], [ %s.3127, %if.end74 ], [ %s.3127, %if.else63 ], [ %incdec.ptr81, %if.end78 ]
  %acc.0.lcssa = phi i32 [ 0, %if.end45 ], [ %acc.0128, %if.end74 ], [ %acc.0128, %if.else63 ], [ %add, %if.end78 ]
  %cmp83.not = icmp eq ptr %endptr, null
  br i1 %cmp83.not, label %if.end86, label %if.then85

if.then85:                                        ; preds = %while.end82
  store ptr %s.3.lcssa, ptr %endptr, align 1, !tbaa !31
  br label %if.end86

if.end86:                                         ; preds = %if.then85, %while.end82
  %sub88 = sub i32 0, %acc.0.lcssa
  %cond89 = select i1 %cmp9.not, i32 %sub88, i32 %acc.0.lcssa
  ret i32 %cond89
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none, target_mem0: none, target_mem1: none)
define dso_local range(i16 0, -32768) i16 @test_rand() local_unnamed_addr #6 {
entry:
  %0 = load i16, ptr @rand_state, align 1, !tbaa !3
  %mul = mul i16 %0, 75
  %add = add i16 %mul, 17
  store i16 %add, ptr @rand_state, align 1, !tbaa !3
  %and = and i16 %add, 32767
  ret i16 %and
}

; Function Attrs: mustprogress nofree norecurse nosync nounwind willreturn memory(write, argmem: none, inaccessiblemem: none, target_mem0: none, target_mem1: none)
define dso_local void @test_srand(i16 noundef %seed) local_unnamed_addr #7 {
entry:
  store i16 %seed, ptr @rand_state, align 1, !tbaa !3
  ret void
}

; Function Attrs: nofree norecurse nounwind memory(readwrite, target_mem0: none, target_mem1: none)
define dso_local noundef i16 @test_putchar(i16 noundef returned %c) local_unnamed_addr #8 {
entry:
  br label %while.cond

while.cond:                                       ; preds = %while.cond, %entry
  %0 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %1 = and i8 %0, 2
  %cmp = icmp eq i8 %1, 0
  br i1 %cmp, label %while.cond, label %while.end, !llvm.loop !41

while.end:                                        ; preds = %while.cond
  %conv2 = trunc i16 %c to i8
  store volatile i8 %conv2, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  ret i16 %c
}

; Function Attrs: nofree norecurse nounwind
define dso_local void @test_printf(ptr noundef readonly captures(none) %fmt, ...) local_unnamed_addr #9 {
entry:
  %ap = alloca ptr, align 1
  call void @llvm.lifetime.start.p0(ptr nonnull %ap) #13
  call void @llvm.va_start.p0(ptr nonnull %ap)
  br label %while.cond

while.cond:                                       ; preds = %while.cond.backedge, %entry
  %fmt.addr.0 = phi ptr [ %fmt, %entry ], [ %fmt.addr.0.be, %while.cond.backedge ]
  %0 = load i8, ptr %fmt.addr.0, align 1, !tbaa !7
  switch i8 %0, label %while.cond.i [
    i8 0, label %while.end68
    i8 37, label %if.end
  ]

while.cond.i:                                     ; preds = %while.cond, %while.cond.i
  %1 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %2 = and i8 %1, 2
  %cmp.i = icmp eq i8 %2, 0
  br i1 %cmp.i, label %while.cond.i, label %test_putchar.exit, !llvm.loop !41

test_putchar.exit:                                ; preds = %while.cond.i
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %fmt.addr.0, i16 1
  store volatile i8 %0, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  br label %while.cond.backedge

while.cond.backedge:                              ; preds = %test_putchar.exit, %if.end66
  %fmt.addr.0.be = phi ptr [ %incdec.ptr, %test_putchar.exit ], [ %incdec.ptr67, %if.end66 ]
  br label %while.cond, !llvm.loop !42

if.end:                                           ; preds = %while.cond
  %incdec.ptr3 = getelementptr inbounds nuw i8, ptr %fmt.addr.0, i16 1
  %3 = load i8, ptr %incdec.ptr3, align 1, !tbaa !7
  switch i8 %3, label %while.cond.i98 [
    i8 100, label %if.then7
    i8 117, label %if.then17
    i8 120, label %if.then23
    i8 88, label %if.then29
    i8 115, label %if.then35
    i8 99, label %if.then47
    i8 37, label %while.cond.i95
  ]

if.then7:                                         ; preds = %if.end
  %4 = va_arg ptr %ap, i16
  %cmp8 = icmp slt i16 %4, 0
  br i1 %cmp8, label %while.cond.i86, label %if.end12

while.cond.i86:                                   ; preds = %if.then7, %while.cond.i86
  %5 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %6 = and i8 %5, 2
  %cmp.i87 = icmp eq i8 %6, 0
  br i1 %cmp.i87, label %while.cond.i86, label %test_putchar.exit88, !llvm.loop !41

test_putchar.exit88:                              ; preds = %while.cond.i86
  store volatile i8 45, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  %sub = sub nsw i16 0, %4
  br label %if.end12

if.end12:                                         ; preds = %if.then7, %test_putchar.exit88
  %u.0 = phi i16 [ %sub, %test_putchar.exit88 ], [ %4, %if.then7 ]
  call fastcc void @print_uint(i16 noundef %u.0, i16 noundef 10, i16 noundef 0) #14
  br label %if.end66

if.then17:                                        ; preds = %if.end
  %7 = va_arg ptr %ap, i16
  call fastcc void @print_uint(i16 noundef %7, i16 noundef 10, i16 noundef 0) #14
  br label %if.end66

if.then23:                                        ; preds = %if.end
  %8 = va_arg ptr %ap, i16
  call fastcc void @print_uint(i16 noundef %8, i16 noundef 16, i16 noundef 0) #14
  br label %if.end66

if.then29:                                        ; preds = %if.end
  %9 = va_arg ptr %ap, i16
  call fastcc void @print_uint(i16 noundef %9, i16 noundef 16, i16 noundef 1) #14
  br label %if.end66

if.then35:                                        ; preds = %if.end
  %10 = va_arg ptr %ap, ptr
  %11 = load i8, ptr %10, align 1, !tbaa !7
  %tobool38.not105 = icmp eq i8 %11, 0
  br i1 %tobool38.not105, label %if.end66, label %while.body39

while.body39:                                     ; preds = %if.then35, %test_putchar.exit91
  %12 = phi i8 [ %15, %test_putchar.exit91 ], [ %11, %if.then35 ]
  %s.0106 = phi ptr [ %incdec.ptr40, %test_putchar.exit91 ], [ %10, %if.then35 ]
  br label %while.cond.i89

while.cond.i89:                                   ; preds = %while.cond.i89, %while.body39
  %13 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %14 = and i8 %13, 2
  %cmp.i90 = icmp eq i8 %14, 0
  br i1 %cmp.i90, label %while.cond.i89, label %test_putchar.exit91, !llvm.loop !41

test_putchar.exit91:                              ; preds = %while.cond.i89
  %incdec.ptr40 = getelementptr inbounds nuw i8, ptr %s.0106, i16 1
  store volatile i8 %12, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  %15 = load i8, ptr %incdec.ptr40, align 1, !tbaa !7
  %tobool38.not = icmp eq i8 %15, 0
  br i1 %tobool38.not, label %if.end66, label %while.body39, !llvm.loop !43

if.then47:                                        ; preds = %if.end
  %16 = va_arg ptr %ap, i16
  br label %while.cond.i92

while.cond.i92:                                   ; preds = %while.cond.i92, %if.then47
  %17 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %18 = and i8 %17, 2
  %cmp.i93 = icmp eq i8 %18, 0
  br i1 %cmp.i93, label %while.cond.i92, label %test_putchar.exit94, !llvm.loop !41

test_putchar.exit94:                              ; preds = %while.cond.i92
  %conv2.i = trunc i16 %16 to i8
  store volatile i8 %conv2.i, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  br label %if.end66

while.cond.i95:                                   ; preds = %if.end, %while.cond.i95
  %19 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %20 = and i8 %19, 2
  %cmp.i96 = icmp eq i8 %20, 0
  br i1 %cmp.i96, label %while.cond.i95, label %test_putchar.exit97, !llvm.loop !41

test_putchar.exit97:                              ; preds = %while.cond.i95
  store volatile i8 37, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  br label %if.end66

while.cond.i98:                                   ; preds = %if.end, %while.cond.i98
  %21 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %22 = and i8 %21, 2
  %cmp.i99 = icmp eq i8 %22, 0
  br i1 %cmp.i99, label %while.cond.i98, label %test_putchar.exit100, !llvm.loop !41

test_putchar.exit100:                             ; preds = %while.cond.i98
  store volatile i8 37, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  %23 = load i8, ptr %incdec.ptr3, align 1, !tbaa !7
  br label %while.cond.i101

while.cond.i101:                                  ; preds = %while.cond.i101, %test_putchar.exit100
  %24 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %25 = and i8 %24, 2
  %cmp.i102 = icmp eq i8 %25, 0
  br i1 %cmp.i102, label %while.cond.i101, label %test_putchar.exit104, !llvm.loop !41

test_putchar.exit104:                             ; preds = %while.cond.i101
  store volatile i8 %23, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  br label %if.end66

if.end66:                                         ; preds = %test_putchar.exit91, %if.then35, %if.then17, %if.then29, %test_putchar.exit94, %test_putchar.exit104, %test_putchar.exit97, %if.then23, %if.end12
  %incdec.ptr67 = getelementptr inbounds nuw i8, ptr %fmt.addr.0, i16 2
  br label %while.cond.backedge

while.end68:                                      ; preds = %while.cond
  call void @llvm.va_end.p0(ptr nonnull %ap)
  call void @llvm.lifetime.end.p0(ptr nonnull %ap) #13
  ret void
}

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn
declare void @llvm.va_start.p0(ptr) #10

; Function Attrs: nofree noinline norecurse nounwind memory(readwrite, target_mem0: none, target_mem1: none)
define internal fastcc void @print_uint(i16 noundef %val, i16 noundef range(i16 10, 17) %base, i16 noundef range(i16 0, 2) %upper) unnamed_addr #11 {
entry:
  %buf = alloca [6 x i8], align 1
  call void @llvm.lifetime.start.p0(ptr nonnull %buf) #13
  %cmp = icmp eq i16 %val, 0
  br i1 %cmp, label %while.cond.i, label %while.cond.preheader

while.cond.preheader:                             ; preds = %entry
  %tobool2.not = icmp eq i16 %upper, 0
  %add3 = select i1 %tobool2.not, i16 87, i16 55
  br label %while.body

while.cond.i:                                     ; preds = %entry, %while.cond.i
  %0 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %1 = and i8 %0, 2
  %cmp.i = icmp eq i8 %1, 0
  br i1 %cmp.i, label %while.cond.i, label %test_putchar.exit, !llvm.loop !41

test_putchar.exit:                                ; preds = %while.cond.i
  store volatile i8 48, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  br label %cleanup

while.body:                                       ; preds = %while.cond.preheader, %cond.end
  %val.addr.025 = phi i16 [ %val, %while.cond.preheader ], [ %div, %cond.end ]
  %i.024 = phi i16 [ 0, %while.cond.preheader ], [ %inc, %cond.end ]
  %val.addr.025.frozen = freeze i16 %val.addr.025
  %div = udiv i16 %val.addr.025.frozen, %base
  %2 = mul i16 %div, %base
  %rem.decomposed = sub i16 %val.addr.025.frozen, %2
  %cmp1 = icmp samesign ult i16 %rem.decomposed, 10
  br i1 %cmp1, label %cond.true, label %cond.false

cond.true:                                        ; preds = %while.body
  %add = or disjoint i16 %rem.decomposed, 48
  br label %cond.end

cond.false:                                       ; preds = %while.body
  %sub = add nuw nsw i16 %add3, %rem.decomposed
  br label %cond.end

cond.end:                                         ; preds = %cond.false, %cond.true
  %cond4 = phi i16 [ %add, %cond.true ], [ %sub, %cond.false ]
  %conv = trunc nuw nsw i16 %cond4 to i8
  %inc = add nuw nsw i16 %i.024, 1
  %arrayidx = getelementptr inbounds nuw i8, ptr %buf, i16 %i.024
  store i8 %conv, ptr %arrayidx, align 1, !tbaa !7
  %tobool.not = icmp ugt i16 %base, %val.addr.025
  br i1 %tobool.not, label %while.body7, label %while.body, !llvm.loop !44

while.body7:                                      ; preds = %cond.end, %test_putchar.exit23
  %i.126 = phi i16 [ %dec, %test_putchar.exit23 ], [ %inc, %cond.end ]
  %dec = add nsw i16 %i.126, -1
  %arrayidx8 = getelementptr inbounds i8, ptr %buf, i16 %dec
  %3 = load i8, ptr %arrayidx8, align 1, !tbaa !7
  br label %while.cond.i21

while.cond.i21:                                   ; preds = %while.cond.i21, %while.body7
  %4 = load volatile i8, ptr inttoptr (i16 -16640 to ptr), align 256, !tbaa !7
  %5 = and i8 %4, 2
  %cmp.i22 = icmp eq i8 %5, 0
  br i1 %cmp.i22, label %while.cond.i21, label %test_putchar.exit23, !llvm.loop !41

test_putchar.exit23:                              ; preds = %while.cond.i21
  store volatile i8 %3, ptr inttoptr (i16 -16639 to ptr), align 1, !tbaa !7
  %tobool6.not = icmp eq i16 %dec, 0
  br i1 %tobool6.not, label %cleanup, label %while.body7, !llvm.loop !45

cleanup:                                          ; preds = %test_putchar.exit23, %test_putchar.exit
  call void @llvm.lifetime.end.p0(ptr nonnull %buf) #13
  ret void
}

; Function Attrs: mustprogress nocallback nofree nosync nounwind willreturn
declare void @llvm.va_end.p0(ptr) #10

; Function Attrs: nocallback nofree nosync nounwind speculatable willreturn memory(none)
declare i32 @llvm.abs.i32(i32, i1 immarg) #12

attributes #0 = { nofree norecurse nosync nounwind memory(argmem: readwrite) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { mustprogress nocallback nofree nosync nounwind willreturn memory(argmem: readwrite) }
attributes #2 = { nofree norecurse nosync nounwind memory(argmem: write) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #3 = { nofree norecurse nosync nounwind memory(argmem: read) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #4 = { nofree norecurse nosync nounwind memory(readwrite, inaccessiblemem: none, target_mem0: none, target_mem1: none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #5 = { mustprogress nofree norecurse nosync nounwind willreturn memory(none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #6 = { mustprogress nofree norecurse nosync nounwind willreturn memory(readwrite, argmem: none, inaccessiblemem: none, target_mem0: none, target_mem1: none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #7 = { mustprogress nofree norecurse nosync nounwind willreturn memory(write, argmem: none, inaccessiblemem: none, target_mem0: none, target_mem1: none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #8 = { nofree norecurse nounwind memory(readwrite, target_mem0: none, target_mem1: none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #9 = { nofree norecurse nounwind "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #10 = { mustprogress nocallback nofree nosync nounwind willreturn }
attributes #11 = { nofree noinline norecurse nounwind memory(readwrite, target_mem0: none, target_mem1: none) "frame-pointer"="all" "no-builtins" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #12 = { nocallback nofree nosync nounwind speculatable willreturn memory(none) }
attributes #13 = { nounwind }
attributes #14 = { nobuiltin "no-builtins" }

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
!10 = distinct !{!10, !9}
!11 = distinct !{!11, !9}
!12 = distinct !{!12, !9}
!13 = distinct !{!13, !9}
!14 = distinct !{!14, !9}
!15 = distinct !{!15, !9}
!16 = distinct !{!16, !9}
!17 = distinct !{!17, !9}
!18 = distinct !{!18, !9}
!19 = distinct !{!19, !9}
!20 = distinct !{!20, !9}
!21 = distinct !{!21, !9}
!22 = distinct !{!22, !9}
!23 = distinct !{!23, !9}
!24 = distinct !{!24, !9}
!25 = distinct !{!25, !9}
!26 = distinct !{!26, !9}
!27 = distinct !{!27, !9}
!28 = distinct !{!28, !9}
!29 = distinct !{!29, !9}
!30 = distinct !{!30, !9}
!31 = !{!32, !32, i64 0}
!32 = !{!"p1 omnipotent char", !33, i64 0}
!33 = !{!"any pointer", !5, i64 0}
!34 = distinct !{!34, !9}
!35 = distinct !{!35, !9}
!36 = distinct !{!36, !9}
!37 = distinct !{!37, !9}
!38 = distinct !{!38, !9}
!39 = distinct !{!39, !9}
!40 = distinct !{!40, !9}
!41 = distinct !{!41, !9}
!42 = distinct !{!42, !9}
!43 = distinct !{!43, !9}
!44 = distinct !{!44, !9}
!45 = distinct !{!45, !9}
