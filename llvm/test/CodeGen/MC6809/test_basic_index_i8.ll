; RUN: llc -verify-machineinstrs -O0 --filetype=asm < %s | FileCheck %s -check-prefixes=CHECK
; RUN: llc -verify-machineinstrs -O0 --filetype=asm -mcpu hd6309 < %s | FileCheck %s -check-prefixes=CHECK-HD6309
target triple = "mc6809"

; Function Attrs: mustprogress nofree norecurse nosync nounwind readonly willreturn
define i8 @foo(i8* nocapture noundef readonly %a, i8 noundef signext %b) #0 {
entry:
  %arrayidx = getelementptr inbounds i8, i8* %a, i8 %b
  %0 = load i8, i8* %arrayidx, align 1, !tbaa !3
  ret i8 %0
}

attributes #0 = { mustprogress nofree norecurse nosync nounwind readonly willreturn "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" }

!llvm.module.flags = !{!0, !1}
!llvm.ident = !{!2}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 7, !"frame-pointer", i32 2}
!2 = !{!"clang version 15.0.0 (https://github.com/llvm-mos/llvm-mos.git 53eca6c20776f15590dd1e3843071012d5b3812d)"}
!3 = !{!4, !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
