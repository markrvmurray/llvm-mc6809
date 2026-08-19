; RUN: llc -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
;
; Hand-authored: a debug value keeps its source variable's own type, so at
; -g a three-bit variable arrives at register-bank selection as three bits.
; No register class models that, and the mapper used to give up on it --
; a picolibc -g build was the way in.  Anything narrower than a byte goes
; in a byte.

target datalayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16"
target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: narrow:
; CHECK: rts
define i8 @narrow(i8 %x) !dbg !4 {
  %t = trunc i8 %x to i3
    #dbg_value(i3 %t, !9, !DIExpression(), !11)
  %r = zext i3 %t to i8
  ret i8 %r
}

!llvm.dbg.cu = !{!0}
!llvm.module.flags = !{!2, !3}

!0 = distinct !DICompileUnit(language: DW_LANG_C11, file: !1, producer: "test", isOptimized: true, runtimeVersion: 0, emissionKind: FullDebug)
!1 = !DIFile(filename: "t.c", directory: "/")
!2 = !{i32 7, !"Dwarf Version", i32 4}
!3 = !{i32 2, !"Debug Info Version", i32 3}
!4 = distinct !DISubprogram(name: "narrow", scope: !1, file: !1, line: 1, type: !5, scopeLine: 1, spFlags: DISPFlagDefinition, unit: !0, retainedNodes: !8)
!5 = !DISubroutineType(types: !6)
!6 = !{!7}
!7 = !DIBasicType(name: "unsigned char", size: 8, encoding: DW_ATE_unsigned_char)
!8 = !{!9}
!9 = !DILocalVariable(name: "mode", scope: !4, file: !1, line: 2, type: !10)
!10 = !DIBasicType(name: "mode_t", size: 3, encoding: DW_ATE_unsigned)
!11 = !DILocation(line: 2, column: 1, scope: !4)
