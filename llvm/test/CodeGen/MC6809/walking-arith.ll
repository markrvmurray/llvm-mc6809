; Hand-authored: a memory operand read through a pointer that steps by the
; access size folds, step and all, into the op that consumes it -- the walking
; forms (`op ,x+`, `op ,x++`, `op ,--x`) -- and an i16 accumulate of a
; zero-extended byte keeps its sum in D (`addb ,x+ ; adca #0`). Both loads
; of a walking byte compare step, one in the load and one in the compare.
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,MC6809
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,HD6309

; x ^= *s++
; CHECK-LABEL: xor8:
; CHECK: eorb ,x+

; x |= *s++ (16-bit): only the HD6309 has a 16-bit OR
; CHECK-LABEL: or16:
; MC6809: orb 1,x
; MC6809-NEXT: ora ,x
; MC6809-NEXT: leax 2,x
; HD6309: ord ,x++

; x += *s++ (16-bit)
; CHECK-LABEL: add16:
; CHECK: addd ,x++

; x -= *--e (16-bit)
; CHECK-LABEL: sub16r:
; CHECK: subd ,--x

; t -= (u16)*s++
; CHECK-LABEL: dsum:
; CHECK: subb ,x+
; CHECK-NEXT: sbca #0
; CHECK-NOT: __rs
; CHECK: rts

; if (*a++ != *b++)
; CHECK-LABEL: walkcmp:
; CHECK: ld{{[bf]}} ,x+
; CHECK-NEXT: cmp{{[bf]}} ,y+
; CHECK-NEXT: bne

; t = (t << 1) + *s++
; CHECK-LABEL: mix:
; CHECK: addb ,x+
; CHECK-NEXT: adca #0

target triple = "mc6809-unknown-unknown"

define dso_local zeroext i8 @xor8(ptr nofree noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not5 = icmp eq i16 %n, 0
  br i1 %tobool.not5, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %x.08 = phi i8 [ %xor4, %while.body ], [ 0, %entry ]
  %n.addr.07 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.06 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %dec = add i16 %n.addr.07, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.06, i16 1
  %0 = load i8, ptr %s.addr.06, align 1, !tbaa !9
  %xor4 = xor i8 %0, %x.08
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !10

while.end:                                        ; preds = %while.body, %entry
  %x.0.lcssa = phi i8 [ 0, %entry ], [ %xor4, %while.body ]
  ret i8 %x.0.lcssa
}

define dso_local i16 @or16(ptr nofree noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %x.05 = phi i16 [ %or, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %dec = add i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.03, i16 2
  %0 = load i16, ptr %s.addr.03, align 1, !tbaa !12
  %or = or i16 %0, %x.05
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !13

while.end:                                        ; preds = %while.body, %entry
  %x.0.lcssa = phi i16 [ 0, %entry ], [ %or, %while.body ]
  ret i16 %x.0.lcssa
}

define dso_local i16 @add16(ptr nofree noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %x.05 = phi i16 [ %add, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %dec = add i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.03, i16 2
  %0 = load i16, ptr %s.addr.03, align 1, !tbaa !12
  %add = add i16 %0, %x.05
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !14

while.end:                                        ; preds = %while.body, %entry
  %x.0.lcssa = phi i16 [ 0, %entry ], [ %add, %while.body ]
  ret i16 %x.0.lcssa
}

define dso_local i16 @sub16r(ptr nofree noundef readonly captures(none) %e, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %x.05 = phi i16 [ %sub, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %e.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %e, %entry ]
  %dec = add i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds i8, ptr %e.addr.03, i16 -2
  %0 = load i16, ptr %incdec.ptr, align 1, !tbaa !12
  %sub = sub i16 %x.05, %0
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !15

while.end:                                        ; preds = %while.body, %entry
  %x.0.lcssa = phi i16 [ 0, %entry ], [ %sub, %while.body ]
  ret i16 %x.0.lcssa
}

define dso_local i16 @dsum(ptr nofree noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %t.05 = phi i16 [ %sub, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %dec = add i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.03, i16 1
  %0 = load i8, ptr %s.addr.03, align 1, !tbaa !9
  %conv = zext i8 %0 to i16
  %sub = sub i16 %t.05, %conv
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !16

while.end:                                        ; preds = %while.body, %entry
  %t.0.lcssa = phi i16 [ 0, %entry ], [ %sub, %while.body ]
  ret i16 %t.0.lcssa
}


define dso_local range(i16 0, 2) i16 @walkcmp(ptr nofree noundef readonly captures(none) %a, ptr nofree noundef readonly captures(none) %b, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not4 = icmp eq i16 %n, 0
  br i1 %tobool.not4, label %return, label %while.body

while.cond:                                       ; preds = %while.body
  %dec = add i16 %n.addr.07, -1
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %b.addr.06, i16 1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %a.addr.05, i16 1
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %return, label %while.body, !llvm.loop !18

while.body:                                       ; preds = %entry, %while.cond
  %n.addr.07 = phi i16 [ %dec, %while.cond ], [ %n, %entry ]
  %b.addr.06 = phi ptr [ %incdec.ptr1, %while.cond ], [ %b, %entry ]
  %a.addr.05 = phi ptr [ %incdec.ptr, %while.cond ], [ %a, %entry ]
  %0 = load i8, ptr %a.addr.05, align 1, !tbaa !9
  %1 = load i8, ptr %b.addr.06, align 1, !tbaa !9
  %cmp.not = icmp eq i8 %0, %1
  br i1 %cmp.not, label %while.cond, label %while.body.return_crit_edge, !llvm.loop !18

while.body.return_crit_edge:                      ; preds = %while.body
  br label %return, !llvm.loop !18

return:                                           ; preds = %while.cond, %while.body.return_crit_edge, %entry
  %retval.0 = phi i16 [ 1, %while.body.return_crit_edge ], [ 0, %entry ], [ 0, %while.cond ]
  ret i16 %retval.0
}

define dso_local i16 @mix(ptr nofree noundef readonly captures(none) %s, i16 noundef %n, i16 noundef %k) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %t.05 = phi i16 [ %add, %while.body ], [ %k, %entry ]
  %s.addr.04 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %n.addr.03 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %dec = add i16 %n.addr.03, -1
  %shl = shl i16 %t.05, 1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.04, i16 1
  %0 = load i8, ptr %s.addr.04, align 1, !tbaa !9
  %conv = zext i8 %0 to i16
  %add = add i16 %shl, %conv
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !19

while.end:                                        ; preds = %while.body, %entry
  %t.0.lcssa = phi i16 [ %k, %entry ], [ %add, %while.body ]
  ret i16 %t.0.lcssa
}

attributes #0 = { nofree norecurse nosync nounwind optsize memory(argmem: read) "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

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
!12 = !{!6, !6, i64 0}
!13 = distinct !{!13, !11}
!14 = distinct !{!14, !11}
!15 = distinct !{!15, !11}
!16 = distinct !{!16, !11}
!17 = distinct !{!17, !11}
!18 = distinct !{!18, !11}
!19 = distinct !{!19, !11}
