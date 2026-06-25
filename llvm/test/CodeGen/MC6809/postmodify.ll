; NOTE: Hand-authored — pins the auto-increment / -decrement post-modify fusion.
; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs -filetype=asm < %s | FileCheck %s
; RUN: llc -mtriple=mc6809 -mcpu=hd6309 -O2 -verify-machineinstrs -filetype=asm < %s | FileCheck %s
;
; A pointer-walk load/store (*s++, *--s) fuses the pointer advance with the
; access into a single post-modify indexed instruction, keeping the pointer in
; one index register instead of copying it to a second register and advancing
; that. The accumulator is reg-agnostic ({{[a-f]}}): HD6309 may place the value
; in E/F (e.g. ldf ,y+ / stf ,x+), which must also have post-modify opcodes.

; *s++ load
; CHECK-LABEL: sumv:
; CHECK: ld{{[a-f]}} ,x+

; *d++ = *s++
; CHECK-LABEL: cpyv:
; CHECK: ld{{[a-f]}} ,y+
; CHECK: st{{[a-f]}} ,x+

; *--e load
; CHECK-LABEL: rsumv:
; CHECK: ld{{[a-f]}} ,-x

; ModuleID = 'pmlit.c'
source_filename = "pmlit.c"
target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local i16 @sumv(ptr noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %t.05 = phi i16 [ %add, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %dec = add nsw i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.03, i16 1
  %0 = load i8, ptr %s.addr.03, align 1, !tbaa !8
  %conv = zext i8 %0 to i16
  %add = add nuw nsw i16 %t.05, %conv
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !9

while.end:                                        ; preds = %while.body, %entry
  %t.0.lcssa = phi i16 [ 0, %entry ], [ %add, %while.body ]
  ret i16 %t.0.lcssa
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: readwrite)
define dso_local void @cpyv(ptr noundef writeonly captures(none) %d, ptr noundef readonly captures(none) %s, i16 noundef %n) local_unnamed_addr #1 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %n.addr.05 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %s.addr.04 = phi ptr [ %incdec.ptr, %while.body ], [ %s, %entry ]
  %d.addr.03 = phi ptr [ %incdec.ptr1, %while.body ], [ %d, %entry ]
  %dec = add nsw i16 %n.addr.05, -1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %s.addr.04, i16 1
  %0 = load i8, ptr %s.addr.04, align 1, !tbaa !8
  %incdec.ptr1 = getelementptr inbounds nuw i8, ptr %d.addr.03, i16 1
  store i8 %0, ptr %d.addr.03, align 1, !tbaa !8
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !11

while.end:                                        ; preds = %while.body, %entry
  ret void
}

; Function Attrs: nofree norecurse nosync nounwind memory(argmem: read)
define dso_local i16 @rsumv(ptr noundef readonly captures(none) %e, i16 noundef %n) local_unnamed_addr #0 {
entry:
  %tobool.not2 = icmp eq i16 %n, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %t.05 = phi i16 [ %add, %while.body ], [ 0, %entry ]
  %n.addr.04 = phi i16 [ %dec, %while.body ], [ %n, %entry ]
  %e.addr.03 = phi ptr [ %incdec.ptr, %while.body ], [ %e, %entry ]
  %dec = add nsw i16 %n.addr.04, -1
  %incdec.ptr = getelementptr inbounds i8, ptr %e.addr.03, i16 -1
  %0 = load i8, ptr %incdec.ptr, align 1, !tbaa !8
  %conv = zext i8 %0 to i16
  %add = add nuw nsw i16 %t.05, %conv
  %tobool.not = icmp eq i16 %dec, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !12

while.end:                                        ; preds = %while.body, %entry
  %t.0.lcssa = phi i16 [ 0, %entry ], [ %add, %while.body ]
  ret i16 %t.0.lcssa
}

attributes #0 = { nofree norecurse nosync nounwind memory(argmem: read) "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }
attributes #1 = { nofree norecurse nosync nounwind memory(argmem: readwrite) "frame-pointer"="all" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0, !1, !2}
!llvm.ident = !{!3}
!llvm.errno.tbaa = !{!4}

!0 = !{i32 8, !"PIC Level", i32 2}
!1 = !{i32 7, !"PIE Level", i32 2}
!2 = !{i32 7, !"frame-pointer", i32 2}
!3 = !{!"clang version 23.0.0git (git@github.com:markrvmurray/llvm-mc6809.git 7e5e4e7c432e83d8f600d5d38bfa95a19a09ca9a)"}
!4 = !{!5, !5, i64 0}
!5 = !{!"int", !6, i64 0}
!6 = !{!"omnipotent char", !7, i64 0}
!7 = !{!"Simple C/C++ TBAA"}
!8 = !{!6, !6, i64 0}
!9 = distinct !{!9, !10}
!10 = !{!"llvm.loop.mustprogress"}
!11 = distinct !{!11, !10}
!12 = distinct !{!12, !10}
